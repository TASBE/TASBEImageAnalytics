from ij import IJ, ImagePlus, WindowManager
from ij.process import ImageConverter, ImageProcessor, ByteProcessor
from ij.plugin import ChannelSplitter, ImageCalculator

from java.lang import Double
import os, glob, re, time, sys

# I'm not certain why, but when run in ImageJ it doesn't seem to adhere to the CLASSPATH env variable
# This ensures that CLASSPATH is explicitly on the module search path, which is required for ELMConfig to resolve
for path in os.environ['CLASSPATH'].split(os.pathsep):
    sys.path.append(path)

import ELMConfig
    
#
#
#
def printUsage():
    global numChannels;
    global numZ;
    
    print "This script will read tif files from an input directory and compute statistics on the cell images."
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
    print "dsNameIdx - Index of well name within filename, when delimiting on underscores (_), also read from XML properties"
    print "wellNames - Optional, list of well names to process, others are ignored"
    print "debugOutput - Optional, True or False, if True output additional info for debugging purposes"

    print "Usage: "
    print "<cfgPath>"
 
 
####
#
#
####
def main(cfg):
    # Input Params
    # TODO: should find a way to input besides hardcoding

    print "Processing input dir " + cfg.getValue(ELMConfig.inputDir);
    print "Outputting in " + cfg.getValue(ELMConfig.outputDir);
    print "\n\n"

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
    # Analyze image filenames to get different pieces of information
    # We care about a time, Z, channel, and the well name
    timeRE = re.compile("^t[0-9]+$")
    zRE    = re.compile("z[0-9]+$")
    chRE   = re.compile("^ch[0-9]+$")
    wellRE   = re.compile("^[a-zA-Z][0-9]+$")
    for filePath in imgFiles:
        fileName = os.path.basename(filePath)
        toks = os.path.splitext(fileName)[0].split("_")
        # Parse file name to get indices of certain values
        tIdx = zIdx = chIdx = sys.maxint
        wellIndex = sys.maxint # This will be the lowest index that matches the well expression
        for i in range(0, len(toks)):
            if timeRE.match(toks[i]):
                tIdx = i
            if zRE.match(toks[i]):
                zIdx = i
            if chRE.match(toks[i]):
                chIdx = i
            if wellRE.match(toks[i]) and i < wellIndex:
                wellIndex = i
        
        minInfoIdx = min(tIdx, min(zIdx, chIdx))
        wellNames.append(toks[wellIndex])
        wellDesc[toks[wellIndex]] = fileName[0:fileName.find(toks[minInfoIdx]) - 1]
        noZInFile[toks[wellIndex]] = zIdx == -1        

    uniqueNames = list(set(wellNames))
    ELMConfig.sort_nicely(uniqueNames)

    # Try to determine pixel size from Leica properties
    metadataDir = os.path.join(cfg.getValue(ELMConfig.inputDir), "MetaData")
    metadataExists = True
    if not os.path.exists(metadataDir):
        print "No MetaData directory in input dir! Can't read properties!"
        metadataExists = False;

    # Process each well
    dsResults = []
    for wellName in uniqueNames:
        # Check to see if we should ignore this well
        if cfg.getValue(ELMConfig.wellNames):
            if not wellName in cfg.getValue(ELMConfig.wellNames):
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
        
        print ("Beginning well " + wellName + "...")
        cfg.printCfg()
        start = time.time()
        processDataset(cfg, wellName, dsImgFiles)
        end = time.time()
        print("Processed well " + wellName + " in " + str(end - start) + " s")
        print("\n\n")


####
#
#
####
def processDataset(cfg, datasetName, imgFiles):
    datasetPath = os.path.join(cfg.getValue(ELMConfig.outputDir), datasetName)

    # Count how many images we have for each channel/Z slice
    imgFileCats = [[[] for z in range(cfg.getValue(ELMConfig.numZ))] for c in range(cfg.getValue(ELMConfig.numChannels))]
    addedImages = False
    for c in range(0, cfg.getValue(ELMConfig.numChannels)):
        chanStr = 'ch%(channel)02d' % {"channel" : c};
        for z in range(0, cfg.getValue(ELMConfig.numZ)):
            zStr = cfg.getZStr(z);
            for imgPath in imgFiles:
                fileName = os.path.basename(imgPath)
                if chanStr in fileName and (cfg.getValue(ELMConfig.noZInFile) or zStr in fileName):
                    addedImages = True
                    imgFileCats[c][z] = imgPath

    # Check for no images
    if not addedImages:
        print "Failed to add any images to chan/z categories! Problem with input dir?"
        quit(1)

    # Process all images
    for c in range(0, cfg.getValue(ELMConfig.numChannels)):
        if (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.SKIP):
            continue;
        processImages(cfg, datasetName, datasetPath, c, imgFileCats[c])


####
#
#  All of the processing that happens for each image
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
    print "\tProcessing channel: " + chanName
    for z in range(0, cfg.getValue(ELMConfig.numZ)):
        zStr = cfg.getZStr(z);
        currIP = IJ.openImage(imgFiles[z])
        origImage = currIP.duplicate();
        if cfg.getValue(ELMConfig.debugOutput):
            WindowManager.setTempCurrentImage(currIP);
            IJ.saveAs('png', os.path.join(wellPath, "Orig_" + wellName + "_" + zStr + "_" + chanStr + ".png"))
        # We need to get to a grayscale image, which will be done differently for different channels
        if (chanName == ELMConfig.BRIGHTFIELD):
            toGray = ImageConverter(currIP)
            toGray.convertToGray8()
            darkBackground = False
        elif (chanName == ELMConfig.BLUE) \
                or (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.RED) \
                or (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.GREEN): #
            chanIdx = 2
            if (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.RED):
                chanIdx = 0
            elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.GREEN):
                chanIdx = 1;
            imgChanns = ChannelSplitter.split(currIP);
            currIP.close()
            currIP = imgChanns[chanIdx];
            darkBackground = True
        elif (chanName == ELMConfig.YELLOW):
            title = currIP.getTitle()
            # Create a new image that consists of the average of the red & green channels
            width = currIP.getWidth();
            height = currIP.getHeight();
            newPix = ByteProcessor(width, height)
            for x in range(0, width) :
                for y in range(0,height) :
                    currPix = currIP.getPixel(x,y);
                    newPix.putPixel(x, y, (currPix[0] + currPix[1]) / 2)
            currIP.close()
            currIP = ImagePlus(title, newPix)
            darkBackground = True
        elif (chanName == ELMConfig.SKIP):
            continue
        WindowManager.setTempCurrentImage(currIP);

        if cfg.getValue(ELMConfig.debugOutput):
            IJ.saveAs('png', os.path.join(wellPath, "Processing_" + wellName + "_" + zStr + "_" + chanStr + ".png"))\

        upperThreshImg = currIP.duplicate()

        currIP.getProcessor().setAutoThreshold("Default", darkBackground, ImageProcessor.NO_LUT_UPDATE)
        threshRange = currIP.getProcessor().getMaxThreshold() - currIP.getProcessor().getMinThreshold()
        if currIP.getType() != ImagePlus.GRAY8 :
            print "\tChannel " + cfg.getValue(ELMConfig.chanLabel)[c] + " is not GRAY8, instead type is %d" % currIP.getType()
        if threshRange > 230:
            print "\t\tZ = " + str(z) + ": Ignored Objects due to threshold range!"
            continue
        IJ.run(currIP, "Convert to Mask", "")
        IJ.run(currIP, "Close-", "")
        
        # Brightfield has an additional thresholding step
        if cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.BRIGHTFIELD:
            if cfg.getValue(ELMConfig.debugOutput):
                IJ.saveAs('png', os.path.join(wellPath, "OrigMask" + wellName + "_" + zStr + "_" + chanStr + ".png"))

            upperThresh = 255 * 0.95
            upperThreshImg.getProcessor().setThreshold(upperThresh, 255, ImageProcessor.NO_LUT_UPDATE)
            IJ.run(upperThreshImg, "Convert to Mask", "")
            IJ.run(upperThreshImg, "Close-", "")
            if cfg.getValue(ELMConfig.debugOutput):
                WindowManager.setTempCurrentImage(upperThreshImg);
                IJ.saveAs('png', os.path.join(wellPath, "UpperThreshMask" + wellName + "_" + zStr + "_" + chanStr + ".png"))

            ic = ImageCalculator()
            compositeMask = ic.run("OR create", currIP, upperThreshImg)
            IJ.run(compositeMask, "Close-", "")
            currIP.close()
            currIP = compositeMask
            WindowManager.setTempCurrentImage(currIP);

        if cfg.getValue(ELMConfig.debugOutput):
            WindowManager.setTempCurrentImage(currIP);
            IJ.saveAs('png', os.path.join(wellPath, "Binary_" + wellName + "_" + zStr + "_" + chanStr + ".png"))

        currProcessor = currIP.getProcessor()
        for x in range(0, currIP.getWidth()) :
            for y in range(0,currIP.getHeight()) :
                if not currProcessor.get(x,y) == 0x00000000:
                    ptX = x * cfg.getValue(ELMConfig.pixelWidth)
                    ptY = y * cfg.getValue(ELMConfig.pixelHeight)
                    ptZ = z * cfg.getValue(ELMConfig.pixelDepth);
                    colorPix = origImage.getPixel(x,y)
                    red   = colorPix[0] 
                    green = colorPix[1]
                    blue  = colorPix[2]
                    if (not cfg.hasValue(ELMConfig.pcloudColorThresh) \
                            or colorPix[chanPixBand] > cfg.getValue(ELMConfig.pcloudColorThresh)):
                        points.append([ptX, ptY, ptZ, red, green, blue])

        currIP.close()
        origImage.close()
        upperThreshImg.close()

    print ""

    resultsFile = open(os.path.join(wellPath, chanName + "_cloud.ply"), "w")
    
    resultsFile.write("ply\n")
    resultsFile.write("format ascii 1.0\n")
    resultsFile.write("element vertex " + str(len(points)) + "\n")
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