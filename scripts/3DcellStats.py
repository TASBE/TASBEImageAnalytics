# Copyright (C) 2011 - 2019, Raytheon BBN Technologies and contributors listed
# in the AUTHORS file in TASBE Flow Analytics distribution's top directory.
#
# This file is part of the TASBE Flow Analytics package, and is distributed
# under the terms of the GNU General Public License, with a linking
# exception, as described in the file LICENSE in the TASBE Image Analysis
# package distribution's top directory.

from ij import IJ, WindowManager

from subprocess import call
import os, glob, re, time, sys

# I'm not certain why, but when run in ImageJ it doesn't seem to adhere to the CLASSPATH env variable
# This ensures that CLASSPATH is explicitly on the module search path, which is required for ELMConfig to resolve
for path in os.environ['CLASSPATH'].split(os.pathsep):
    sys.path.append(path)

import ELMConfig, ELMImageUtils
    
#
#
#
def printUsage():
    global numChannels;
    global numZ;
    
    print "This script will read tif files from an input directory and compute statistics on 3D point clouds."
    print "The script must be pointed to a configuration ini file that will define several important aspects."
    print "The input and output dirs must be defined in the config file, however all of the rest of the config"
    print " can be read in from the microscope properties if they exist in the Metadata dir in the input."
    print "The following parameters are recognized in the [Config] section:"

    print "inputDir  - Dir to read in tif files from. (Required)"
    print "outputDir - Dir to write output to. (Required)"
    print "numChannels - Number of channels in dataset, defaults to 4, also read from XML properties"
    print "numZ - Number of Z slices in dataset, defaults to 1, also read from XML properties"
    print "noZInFile - True if the z slice does not appear in the filename, otherwise false, also read from XML properties"
    print "chanLabel - Label that describes source for each channel, default skip, yellow, blue, brightfield, also read from XML properties"
    print "            Valid labels: skip, brightfield, red, green, blue, yellow"
    print "chansToSkip - List of channel names that will be skipped if channels are read from XML properties"
    print "analysisRoi - Rectangular area to perform cell detection on, default 0,0,512,480, must be defined in config file"
    print "wellNames - Optional, list of well names to process, others are ignored"
    print "debugOutput - Optional, True or False, if True output additional info for debugging purposes"

    print "Usage: "
    print "<cfgPath>"
 
 
####
#
#
####
def main(cfg):
    print "Processing input dir " + cfg.getValue(ELMConfig.inputDir);
    print "Outputting in " + cfg.getValue(ELMConfig.outputDir);
    print ""

    # Get all images in the input dir
    imgFiles = glob.glob(os.path.join(cfg.getValue(ELMConfig.inputDir), "*.tif"))
    # Ensure we have tifs
    if (len(imgFiles) < 1):
        print "No tif files found in input directory!  Input dir: " + cfg.getValue(ELMConfig.inputDir)
        quit(1)

    # Sort filenames so they are in order by z and ch
    ELMConfig.sort_nicely(imgFiles)

# Get the names of all wells that exist in this dataset/plate
    wellNames = []
    # Each well will have a collection of images, but will all fall under the same common prefix descriptor
    # Such as: plate1_Aug284pm_A1_S001
    wellDesc = dict()
    noZInFile = dict()
    noTInFile = dict()
    maxT = dict()
    minT = dict()
    numZ = dict()

    # Analyze image filenames to get different pieces of information
    # We care about a time, Z, channel, and the well name
    timeRE = re.compile("^t[0-9]+$")
    zRE    = re.compile("z[0-9]+$")
    chRE   = re.compile("^ch[0-9]+$")
    wellRE = re.compile("^[A-Z][0-9]+$")
    for filePath in imgFiles:
        fileName = os.path.basename(filePath)
        toks = os.path.splitext(fileName)[0].split("_")

        # Parse file name to get indices of certain values
        tIdx = zIdx = chIdx = sys.maxint
        if (cfg.hasValue(ELMConfig.wellIdx)) :
            wellIndex = cfg.getValue(ELMConfig.wellIdx)
        else:
            wellIndex = sys.maxint # This will be the lowest index that matches the well expression
        for i in range(0, len(toks)):
            if timeRE.match(toks[i]):
                tIdx = i
            if zRE.match(toks[i]):
                zIdx = i
            if chRE.match(toks[i]) or toks[i] in cfg.getValue(ELMConfig.chanLabel):
                chIdx = i
            if not cfg.hasValue(ELMConfig.wellIdx) and wellRE.match(toks[i]) and i < wellIndex:
                wellIndex = i

        minInfoIdx = min(tIdx, min(zIdx, chIdx))
        if isinstance(wellIndex, list):
            wellName = ""
            for idx in wellIndex:
                wellName += toks[idx] + "_"
            wellName = wellName[0:len(wellName) - 1]
        else:
            wellName = toks[wellIndex]
        wellNames.append(wellName)
        # Se well description, usd for finding Lyca property files
        if not minInfoIdx == sys.maxint:
            wellDesc[wellName] = fileName[0:fileName.find(toks[minInfoIdx]) - 1]
        # Determine if filename contains z or t info
        noZInFile[wellName] = zIdx == sys.maxint
        noTInFile[wellName] = tIdx == sys.maxint
        if not tIdx == sys.maxint:
            cfg.setValue(ELMConfig.tIdx, tIdx)
            timeTok = toks[tIdx]
            if 't' in timeTok:
                timeTok = timeTok.replace('t','')
            timestep = int(timeTok)
            if wellName not in maxT or timestep > maxT[wellName]:
                maxT[wellName] = timestep
            if wellName not in minT or timestep < minT[wellName]:
                minT[wellName] = timestep
        # Set channel file index
        if not chIdx == sys.maxint:
            cfg.setValue(ELMConfig.cIdx, chIdx)
        # Set Z file index
        if not zIdx == sys.maxint:
            cfg.setValue(ELMConfig.zIdx, zIdx)

    uniqueNames = list(set(wellNames))
    ELMConfig.sort_nicely(uniqueNames)

    # Try to determine pixel size from Leica properties
    metadataDir = os.path.join(cfg.getValue(ELMConfig.inputDir), "MetaData")
    metadataExists = True
    if not os.path.exists(metadataDir):
        print "No MetaData directory in input dir! Can't read properties!"
        metadataExists = False;

    # Process each well
    for wellName in uniqueNames:
        # Check to see if we should ignore this well
        if cfg.getValue(ELMConfig.wellNames):
            if not wellName in cfg.getValue(ELMConfig.wellNames):
                continue;
        if cfg.getValue(ELMConfig.excludeWellNames):
            if any(wellName in name for name in cfg.getValue(ELMConfig.excludeWellNames)):
                continue;

        dsImgFiles = [];
        for i in range(0, len(imgFiles)) :
            if wellName == wellNames[i] :
                dsImgFiles.append(imgFiles[i])
        # Make sure output dir exists for well
        wellPath = os.path.join(cfg.getValue(ELMConfig.outputDir), wellName)
        if not os.path.exists(wellPath):
            os.makedirs(wellPath)

        # Update config based on metadata
        if (metadataExists):
            xmlFile = os.path.join(metadataDir, wellDesc[wellName] + "_Properties.xml")
            if not os.path.exists(xmlFile):
                print "No metadata XML file for well " + wellName + "! Skipping well.  Path: " + xmlFile
                continue;
            cfg.updateCfgWithXML(xmlFile)
            cfg.setValue(ELMConfig.noZInFile, noZInFile[wellName] or cfg.getValue(ELMConfig.numZ) == 1)

        if wellName in maxT:
            cfg.setValue(ELMConfig.numT, maxT[wellName] - minT[wellName] + 1)
        if wellName in minT:
            cfg.setValue(ELMConfig.minT, minT[wellName])

        cfg.setValue(ELMConfig.noZInFile, noZInFile[wellName] or cfg.getValue(ELMConfig.numZ) == 1)
        cfg.setValue(ELMConfig.noTInFile, noTInFile[wellName] or cfg.getValue(ELMConfig.numT) == 1)

        print ("Beginning well " + wellName + "...")
        cfg.printCfg()
        start = time.time()
        processDataset(cfg, wellName, dsImgFiles)
        end = time.time()
        print("Processed well " + wellName + " in " + str(end - start) + " s")
        print("")

####
#
#
####
def processDataset(cfg, datasetName, imgFiles):
    datasetPath = os.path.join(cfg.getValue(ELMConfig.outputDir), datasetName)
    # Categorize images based on c/z/t
    imgFileCats = [[[[] for t in range(cfg.getValue(ELMConfig.numT))] for z in range(cfg.getValue(ELMConfig.numZ))] for c in range(cfg.getValue(ELMConfig.numChannels))]
    for imgPath in imgFiles:
        fileName = os.path.basename(imgPath)
        c,z,t = cfg.getCZTFromFilename(fileName)
        #print "c,z,t: " + str(c) + ", " + str(z) + ", "+ str(t)
        imgFileCats[c][z][t].append(imgPath)
        if (len(imgFileCats[c][z][t]) > 1):
            print "ERROR: More than one image for c,z,t: " + str(c) + ", " + str(z) + ", "+ str(t)
            quit(-1)

    # Check that we have an image for each category
    missingImage = False
    for t in range(cfg.getValue(ELMConfig.numT)):
        for z in range(cfg.getValue(ELMConfig.numZ)):
            for c in range(cfg.getValue(ELMConfig.numChannels)):
                if cfg.getValue(ELMConfig.chanLabel)[c] in cfg.getValue(ELMConfig.chansToSkip):
                    continue;
                if not imgFileCats[c][z][t]:
                    print "ERROR: No image for c,z,t: " + str(c) + ", " + str(z) + ", "+ str(t)
                    missingImage = True
    if missingImage:
        quit(-1)

    # Process all images
    for c in range(0, cfg.getValue(ELMConfig.numChannels)):
        if (cfg.getValue(ELMConfig.chanLabel)[c] in cfg.getValue(ELMConfig.chansToSkip)):
            continue;
        processImages(cfg, datasetName, datasetPath, c, imgFileCats[c])


####
#
#  Process all images for a particular channel.
#
####
def processImages(cfg, wellName, wellPath, c, imgFiles):

    points = []
    chanStr = 'ch%(channel)02d' % {"channel" : c};
    chanName = cfg.getValue(ELMConfig.chanLabel)[c]
    if (chanName == ELMConfig.RED):
        chanPixBand = 0;
    elif (chanName == ELMConfig.GREEN):
        chanPixBand = 1;
    elif (chanName == ELMConfig.BLUE):
        chanPixBand = 2;
    elif (chanName == ELMConfig.YELLOW):
        chanPixBand = 0;
    else:
        chanPixBand = -1;

    chanPixBand
    numExclusionPts = 0
    numColorThreshPts = 0
    ptCount = 0
    print "\tProcessing channel: " + chanName
    for t in range(cfg.getValue(ELMConfig.numT)):
        tStr = cfg.getTStr(t)
        for z in range(0, cfg.getValue(ELMConfig.numZ)):
            zStr = cfg.getZStr(z);
            currIP = IJ.openImage(imgFiles[z][t][0])
            origImage = currIP.duplicate();
            if cfg.getValue(ELMConfig.debugOutput):
                WindowManager.setTempCurrentImage(currIP);
                IJ.saveAs('png', os.path.join(wellPath, "Orig_" + wellName + "_" + zStr + "_" + chanStr + ".png"))
                
            # We need to get to a grayscale image, which will be done differently for different channels
            dbgOutDesc = wellName + "_" + tStr + "_" + zStr + "_" + chanStr
            currIP = ELMImageUtils.getThresholdedMask(currIP, c, z, 1, chanName, cfg, wellPath, dbgOutDesc)
            if (not currIP) :
                continue
    
            currProcessor = currIP.getProcessor()
            #WindowManager.setTempCurrentImage(currIP);
            #currIP.show()
            imgWidth = currIP.getWidth()
            imgHeight = currIP.getHeight()
            for x in range(0, imgWidth) :
                for y in range(0,imgHeight) :
                    if not currProcessor.get(x,y) == 0x00000000:
                        ptCount += 1
                        ptX = x * cfg.getValue(ELMConfig.pixelWidth)
                        ptY = y * cfg.getValue(ELMConfig.pixelHeight)
                        ptZ = z * cfg.getValue(ELMConfig.pixelDepth);
                        colorPix = origImage.getPixel(x,y)
                        red   = colorPix[0]
                        green = colorPix[1]
                        blue  = colorPix[2]
                        # Check that point meets color threshold
                        aboveColorThresh = not cfg.hasValue(ELMConfig.pcloudColorThresh) \
                            or colorPix[chanPixBand] > cfg.getValue(ELMConfig.pcloudColorThresh)
                        # Check that point isn't in exclusion zone
                        outsideExclusion = not (cfg.hasValue(ELMConfig.pcloudExclusionX) and cfg.hasValue(ELMConfig.pcloudExclusionY)) \
                            or (x < cfg.getValue(ELMConfig.pcloudExclusionX) or y < cfg.getValue(ELMConfig.pcloudExclusionY))
    
                        if (aboveColorThresh and outsideExclusion):
                            points.append([ptX, ptY, ptZ, red, green, blue])
                        elif (not aboveColorThresh):
                            numColorThreshPts += 1
                        elif (not outsideExclusion):
                            numExclusionPts += 1
    
            currIP.close()
            origImage.close()
    
        print "\t\tTotal points considered: " + str(ptCount)
        print "\t\tColor Threshold Skipped " + str(numColorThreshPts) + " points."
        print "\t\tExclusion Zone  Skipped " + str(numExclusionPts) + " points."
    
        numPoints = len(points);
        cloudName = chanName + "_" + tStr + "_cloud.ply"
        resultsFile = open(os.path.join(wellPath, cloudName), "w")
        
        resultsFile.write("ply\n")
        resultsFile.write("format ascii 1.0\n")
        resultsFile.write("element vertex " + str(numPoints) + "\n")
        resultsFile.write("property float x\n")
        resultsFile.write("property float y\n")
        resultsFile.write("property float z\n")
        resultsFile.write("property uchar red\n")
        resultsFile.write("property uchar green\n")
        resultsFile.write("property uchar blue\n")
        resultsFile.write("end_header\n")
        for line in points:
            resultsFile.write("%f %f %f %d %d %d\n" % (line[0], line[1], line[2], line[3], line[4], line[5]))
        resultsFile.close()

        if numPoints > 0:
            compute3DStats(cfg, wellPath, wellName, chanName, cloudName, imgWidth, imgHeight)
        else:
            print('Well %s, channel %s (%s) - Skipping 3D stats since we have no points!' % (wellName, chanName, chanStr))

    print ""

####
#
#  Use the saved pointcloud to compute stats
#
####    
def compute3DStats(cfg, wellPath, wellName, chanName, cloudName, imageWidth, imageHeight):
    # Create SegParams INI file
    segIniPath = os.path.join(wellPath, chanName + "_segParams.ini")
    segIniFile = open(segIniPath, "w")
    segIniFile.write("[InputParameters]\n")
    segIniFile.write("RunName=" + wellName + "_" + chanName + "Euc" + "\n")
    segIniFile.write("InputCloud=" + os.path.join(wellPath, cloudName) + "\n")
    segIniFile.write("OutputDir=" + os.path.join(wellPath, chanName + "Seg") + "\n")
    if cfg.hasValue(ELMConfig.scopeProperties):
        segIniFile.write("MicroscopeProperties=" + cfg.getValue(ELMConfig.scopeProperties) + "\n")
    else:
        with open(os.path.join(wellPath, 'scopeProperties.ini'), "w") as resultsFile:
            resultsFile.write("[Config]\n")
            resultsFile.write("pixelHeight=%f\n" % cfg.getValue(ELMConfig.pixelHeight))
            resultsFile.write("pixelWidth=%f\n" % cfg.getValue(ELMConfig.pixelWidth))
            resultsFile.write("pixelDepth=%f\n" % cfg.getValue(ELMConfig.pixelDepth))
            resultsFile.write("imageHeight=%f\n" % imageHeight)
            resultsFile.write("imageWidth=%f\n" % imageWidth)
        segIniFile.write("MicroscopeProperties=" + os.path.join(wellPath, 'scopeProperties.ini') + "\n")

    segIniFile.write("[SegmentationParameters]\n")
    segIniFile.write("SegType=Euclidean\n")
    segIniFile.write("EucClusterTolerance = 12\n")
    segIniFile.write("MinClusterSize = 5\n")
    segIniFile.close()

    elmSegBin = cfg.getValue(ELMConfig.elmSegPath)
    call([elmSegBin, segIniPath])


####
#
#
####
# Checking for __main__ will cause running from ImageJ to fail        
#if __name__ == "__main__":

#@String cfgPath

# Check to see if cfgPath is defined
# They could be defined if running from ImageJ directly
try:
    cfgPath
except NameError:
    argc = len(sys.argv) - 1
    if not argc == 1:
        print "Expected 1 argument, received " + str(argc) + "!"
        printUsage()
        quit(1)
    cfgPath = sys.argv[1]
    
print("***************************************************************")    
# Load the configuration file
cfg = ELMConfig.ConfigParams()
rv = cfg.loadConfig(cfgPath)
if not rv:
    quit(1)

# Print Config
cfg.printCfg()

start = time.time()
main(cfg)
end = time.time()

print("Processed all images in " + str(end - start) + " s")
print("***************************************************************")
print("\n\n")