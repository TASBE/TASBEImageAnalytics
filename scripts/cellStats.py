from ij import IJ, ImagePlus, WindowManager
from ij.process import ImageConverter, FloodFiller
from ij.measure import ResultsTable

from ij.plugin import ChannelSplitter, LutLoader
from ij.plugin.filter import ParticleAnalyzer, Analyzer
from ij.measure import Measurements

from java.lang import Double
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
    
    print "This script will read tif or png files from an input directory and compute statistics on the cell images."
    print "The script must be pointed to a configuration ini file that will define several important aspects."
    print "The input and output dirs must be defined in the config file, however all of the rest of the config"
    print " can be read in from the microscope properties."
    print " For Leica microscopes, the Metadata directory is checked for properties."
    print " For Cytation microscopes, properties are read from tif tags."
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
def getCSVHeader(cfg):
    outputChans = [];
    for chan in cfg.getValue(ELMConfig.chanLabel):
        if not chan in cfg.getValue(ELMConfig.chansToSkip) and not chan == ELMConfig.BRIGHTFIELD:
            outputChans.append(chan)
    headerString = "well, z, t, brightfield area (um^2), "
    for chan in outputChans:
        headerString += chan + " num clusters, "
        headerString += chan + " area (um^2), "
        headerString += "percent " + chan + ", "
    headerString += "classification \n"
    return headerString



####
#
#
####
def main(cfg):
    print "Processing input dir " + cfg.getValue(ELMConfig.inputDir);
    print "Outputting in " + cfg.getValue(ELMConfig.outputDir);
    print "\n\n"

    # Get all images in the input dir
    imgFiles = glob.glob(os.path.join(cfg.getValue(ELMConfig.inputDir), "*." + cfg.getValue(ELMConfig.imgType)))
    # Ensure we have tifs
    if (len(imgFiles) < 1):
        print "No " + cfg.getValue(ELMConfig.imgType) + " files found in input directory!  Input dir: " + cfg.getValue(ELMConfig.inputDir)
        quit(1)

    # Sort filenames so they are in order by z and ch
    ELMConfig.sort_nicely(imgFiles)

    # Check for Cytation metadata
    cfg.checkCytationMetadata(imgFiles[0])
    # If we have cytation data, we need to scan whole set in order to get all
    # of the channel names, unless they are already specified
    if cfg.isCytation:
        if not cfg.hasValue(ELMConfig.chanLabel):
            cfg.getCytationChanNames(imgFiles)

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
    pngTimesteps = dict()
    pngZSlices = dict()

    # Analyze image filenames to get different pieces of information
    # We care about a time, Z, channel, and the well name
    timeRE = re.compile("^t[0-9]+$")
    zRE    = re.compile("z[0-9]+$")
    chRE   = re.compile("^ch[0-9]+$")
    wellRE = re.compile("^[A-Z][0-9]+$")
    posRE  = re.compile("^Pos[0-9]+$")
    for filePath in imgFiles:
        fileName = os.path.basename(filePath)
        toks = os.path.splitext(fileName)[0].split("_")
        
        if (cfg.getValue(ELMConfig.imgType) == "png") :
            if not cfg.hasValue(ELMConfig.wellIdx):
                print "wellIdx not defined, required for png type!"
                quit(-1)
            if not cfg.hasValue(ELMConfig.zIdx):
                print "zIdx not defined, required for png type!"
                quit(-1)
            if not cfg.hasValue(ELMConfig.tIdx):
                print "tIdx not defined, required for png type!"
                quit(-1)

            wellIndex = cfg.getValue(ELMConfig.wellIdx)
            zIdx = cfg.getValue(ELMConfig.zIdx)
            tIdx = cfg.getValue(ELMConfig.tIdx)
            chIdx = sys.maxint
        else :
            # Parse file name to get indices of certain values
            tIdx = zIdx = chIdx = sys.maxint
            if (cfg.hasValue(ELMConfig.wellIdx)) :
                wellIndex = cfg.getValue(ELMConfig.wellIdx)
            else:
                wellIndex = sys.maxint # This will be the lowest index that matches the well expression
            # On the Cytation scope, time is the last token
            if cfg.isCytation:
                tIdx = len(toks) - 1
            for i in range(0, len(toks)):
                if timeRE.match(toks[i]):
                    tIdx = i
                if zRE.match(toks[i]):
                    zIdx = i
                if chRE.match(toks[i]) or toks[i] in cfg.getValue(ELMConfig.chanLabel):
                    chIdx = i
                if not cfg.hasValue(ELMConfig.wellIdx) and (wellRE.match(toks[i]) or posRE.match(toks[i])) and i < wellIndex:
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
        noZInFile[wellName] = zIdx < 0
        noTInFile[wellName] = tIdx < 0
        # Special handling of Z/T info for PNGs
        if (cfg.getValue(ELMConfig.imgType) == "png") :
            timestep = float(toks[tIdx])
            if wellName not in pngTimesteps:
                pngTimesteps[wellName] = set()
            pngTimesteps[wellName].add(timestep)
            maxT[wellName] = len(pngTimesteps[wellName])
            minT[wellName] = 1

            if not noZInFile[wellName]:
                zSlice = float(toks[zIdx])
                if wellName not in pngZSlices:
                    pngZSlices[wellName] = set()
                pngZSlices[wellName].add(zSlice)
                numZ[wellName] = len(pngZSlices[wellName])
        # Update min/max time info
        elif not tIdx == sys.maxint:
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
    if not os.path.exists(metadataDir) and not cfg.isCytation and not (cfg.getValue(ELMConfig.imgType) == "png"):
        print("No MetaData directory in input dir! Can't read Leica properties!")
        metadataExists = False;
    elif cfg.isCytation or (cfg.getValue(ELMConfig.imgType) == "png"):
        metadataExists = False;

    # Process each well
    dsResults = []
    for wellName in uniqueNames:
        # Check to see if we should ignore this well
        if cfg.getValue(ELMConfig.wellNames):
            if not any(wellName in name for name in cfg.getValue(ELMConfig.wellNames)):
                continue;
        if cfg.getValue(ELMConfig.excludeWellNames):
            if any(wellName in name for name in cfg.getValue(ELMConfig.excludeWellNames)):
                continue;

        # Get files in this well
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

        if wellName in maxT:
            cfg.setValue(ELMConfig.numT, maxT[wellName] - minT[wellName] + 1)
        if wellName in minT:
            cfg.setValue(ELMConfig.minT, minT[wellName])

        # Set special properties for PNG images
        if (cfg.getValue(ELMConfig.imgType) == "png"):
            timesteps = list(pngTimesteps[wellName])
            timesteps.sort()
            cfg.setValue(ELMConfig.tList, timesteps)
            if noZInFile[wellName]:
                cfg.setValue(ELMConfig.numZ, 1)
                cfg.setValue(ELMConfig.zList, [0])
            else:
                cfg.setValue(ELMConfig.numZ, numZ[wellName])
                zSlices = list(pngZSlices[wellName]);
                zSlices.sort()
                cfg.setValue(ELMConfig.zList, zSlices)

        cfg.setValue(ELMConfig.noZInFile, noZInFile[wellName] or cfg.getValue(ELMConfig.numZ) == 1)
        cfg.setValue(ELMConfig.noTInFile, noTInFile[wellName] or cfg.getValue(ELMConfig.numT) == 1)

        print ("Beginning well " + wellName + "...")
        cfg.printCfg()
        start = time.time()
        dsResults.append(processDataset(cfg, wellName, dsImgFiles))
        end = time.time()
        print("Processed well " + wellName + " in " + str(end - start) + " s")
        print("\n\n")

    # Write out summary output
    resultsFile = open(os.path.join(cfg.getValue(ELMConfig.outputDir), "AllResults.csv"), "w")

    resultsFile.write(getCSVHeader(cfg))
    for result in dsResults:
        resultsFile.write(result);
    resultsFile.close()



####
#
#
####
def processDataset(cfg, datasetName, imgFiles):
    datasetPath = os.path.join(cfg.getValue(ELMConfig.outputDir), datasetName)

    startTime = time.time()
    # Categorize images based on c/z/t
    imgFileCats = [[[[] for t in range(cfg.getValue(ELMConfig.numT))] for z in range(cfg.getValue(ELMConfig.numZ))] for c in range(cfg.getValue(ELMConfig.numChannels))]
    if cfg.params[ELMConfig.imgType] == "png":
        for imgPath in imgFiles:
            fileName = os.path.basename(imgPath)
            z, t = cfg.getZTFromFilename(fileName)
            for c in range(0, cfg.getValue(ELMConfig.numChannels)):
                imgFileCats[c][z][t].append(imgPath)
                if (len(imgFileCats[c][z][t]) > 1):
                    print "ERROR: More than one image for c,z,t: " + str(c) + ", " + str(z) + ", "+ str(t)
                    quit(-1)
    else:
        for imgPath in imgFiles:
            fileName = os.path.basename(imgPath)
            c,z,t = cfg.getCZTFromFilename(fileName)
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

    fileTime = time.time()
    # Process images
    stats = processImages(cfg, datasetName, datasetPath, imgFileCats)

    statsTime = time.time()
    
    outputChans = [];
    for chan in cfg.getValue(ELMConfig.chanLabel):
        if not chan in cfg.params[ELMConfig.chansToSkip] and not chan == ELMConfig.BRIGHTFIELD:
            outputChans.append(chan)

    # Output Results
    resultsFile = open(os.path.join(cfg.getValue(ELMConfig.outputDir), datasetName + "_results.csv"), "w")
    resultsFile.write(getCSVHeader(cfg));
    resultsString = ""
    for z in range(0, cfg.getValue(ELMConfig.numZ)):
        for t in range(0, cfg.getValue(ELMConfig.numT)):
            channelAreas = dict()
            channelAreas["totalArea"] = 0
            for c in range(0, cfg.getValue(ELMConfig.numChannels)):
                area = 0;
                writeStats = False
                # Skip channel
                if (cfg.getValue(ELMConfig.chanLabel)[c] in cfg.params[ELMConfig.chansToSkip]):
                    continue
                # Handle brightfield channel
                elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.BRIGHTFIELD):
                    if not stats[c][z][t][ELMConfig.UM_AREA] :
                        area = 0;
                    else:
                        area = sum(stats[c][z][t][ELMConfig.UM_AREA])
                        writeStats = True
                    channelAreas["totalArea"] = area
                # Handle Fluorescent Channels   
                elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.BLUE) \
                        or (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.RED) \
                        or (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.GREEN) \
                        or (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.YELLOW): #
                    if not stats[c][z][t][ELMConfig.UM_AREA]:
                        area = 0;
                    else:
                        area = sum(stats[c][z][t][ELMConfig.UM_AREA])
                        writeStats = True
                    channelAreas[cfg.getValue(ELMConfig.chanLabel)[c]] = area
                else:
                    print "ERROR! Unknown channel!"
                    quit(1)
                # Write out blob stats per channel
                if writeStats:
                    chanStr = '_' + cfg.getCStr(c)
                    zStr = '_' + cfg.getZStr(z)
                    tStr = '_' + cfg.getTStr(t)
                    chanResultsFile = open(os.path.join(datasetPath, datasetName + chanStr + zStr + tStr + "_stats.csv"), "w")
                    numParticles = len(stats[c][z][t][ELMConfig.UM_AREA])
                    # Writer Header
                    keys = sorted(stats[c][z][t].keys())
                    headerKeys =  [ key.replace("%", "percent ") for key in keys ]
                    chanResultsFile.write(", ".join(headerKeys) + "\n")
                    for particle in range(0, numParticles):
                        line = ""
                        for measure in keys:
                            if stats[c][z][t][measure]:
                                line += "%10.4f, " % stats[c][z][t][measure][particle]
                            else:
                                line += "      N/A, "
                        line = line[0:len(line)-2] + "\n"
                        chanResultsFile.write(line);
                    chanResultsFile.close()

            # Generate output string for each z/t combo
            zStr = cfg.getZStr(z)
            tStr = cfg.getTStr(t)
            resultsString += datasetName + ", " + zStr + ", " + tStr + ", "
            resultsString += "\t\t\t %10.4f," % (channelAreas["totalArea"])
            numChans = len(outputChans)
            for i in range(0, numChans):
                if channelAreas["totalArea"] == 0:
                    if channelAreas[outputChans[i]] == 0:
                        totalArea = 1
                    else:
                        totalArea = channelAreas[outputChans[i]]
                else:
                    totalArea = channelAreas["totalArea"]
                if not ELMConfig.UM_AREA in stats[i][z][t]:
                    numParticles = -1;
                else:
                    numParticles = len(stats[i][z][t][ELMConfig.UM_AREA])
                resultsString += "\t\t %d," % numParticles
                resultsString += "\t\t %10.4f," % channelAreas[outputChans[i]]
                resultsString += "\t\t %0.4f" % (channelAreas[outputChans[i]] / totalArea)
                if (i + 1 < numChans):
                    resultsString += ","
            resultsString += "\n"

    resultsFile.write(resultsString)
    resultsFile.close()
    
    outputTime = time.time();
    print("Well times: fileTime: %f, statsTime: %f, outputTime: %f" % (fileTime - startTime, statsTime-fileTime, outputTime - statsTime))
    return resultsString



####
#
#  All of the processing that happens for each image
#
####
def processImages(cfg, wellName, wellPath, images):

    stats = [[[dict() for t in range(cfg.getValue(ELMConfig.numT))] for z in range(cfg.getValue(ELMConfig.numZ))] for c in range(cfg.getValue(ELMConfig.numChannels))]
    times = {}
    for c in range(0, cfg.getValue(ELMConfig.numChannels)):
        chanStr = 'ch%(channel)02d' % {"channel" : c};
        chanName = cfg.getValue(ELMConfig.chanLabel)[c]

        # Set some config based upon channel
        if (cfg.getValue(ELMConfig.chanLabel)[c] in cfg.getValue(ELMConfig.chansToSkip)):
            continue
        if (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.BRIGHTFIELD):
            minCircularity = 0.001 # We want to identify one big cell ball, so ignore small less circular objects
            if cfg.params[ELMConfig.imgType] == "png":
                minSize = 5;
            else:
                minSize = 500
        elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.BLUE) \
                or (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.RED) \
                or (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.GREEN): #
            minCircularity = 0.001
            minSize = 5
        elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.YELLOW):
            minCircularity = 0.001
            minSize = 5

        # Process images in Z stack
        for z in range(0, cfg.getValue(ELMConfig.numZ)):
            zStr = cfg.getZStr(z);
            for t in range(0, cfg.getValue(ELMConfig.numT)):
                tStr = cfg.getTStr(t)
                if (cfg.getValue(ELMConfig.imgType) == "png"):
                    # Brightfield uses the whole iamge
                    if (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.BRIGHTFIELD):
                        currIP = IJ.openImage(images[c][z][t][0])
                    else: # otherwise, we'll plit off channels
                        chanIdx = 2
                        if (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.RED):
                            chanIdx = 0
                        elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.GREEN):
                            chanIdx = 1;
                        img = IJ.openImage(images[c][z][t][0])
                        imgChanns = ChannelSplitter.split(img);
                        img.close()
                        currIP = imgChanns[chanIdx];
                else:
                    currIP = IJ.openImage(images[c][z][t][0])
                resultsImage = currIP.duplicate()
                dbgOutDesc = wellName + "_" + zStr + "_" + chanStr + "_" + tStr
                if (cfg.getValue(ELMConfig.numT) > 1):
                    outputPath = os.path.join(wellPath, "images") 
                    if not os.path.exists(outputPath):
                        os.makedirs(outputPath)
                else:
                    outputPath = wellPath

                if cfg.getValue(ELMConfig.debugOutput):
                    WindowManager.setTempCurrentImage(currIP)
                    IJ.saveAs('png', os.path.join(outputPath, "Orig_" + dbgOutDesc +  ".png"))

                # We need to get to a grayscale image, which will be done differently for different channels
                startTime = time.time()
                currIP = ELMImageUtils.getThresholdedMask(currIP, c, z, t, chanName, cfg, outputPath, dbgOutDesc)
                endTime = time.time()
                if not 'grayscale' in times:
                    times['grayscale'] = []
                times['grayscale'].append(endTime-startTime)

                if (not currIP):
                    resultsImage.close()
                    stats[c][z][t][ELMConfig.UM_AREA] = []
                    continue
                
                startTime = time.time()
                # Create a table to store the results
                table = ResultsTable()
                # Create a hidden ROI manager, to store a ROI for each blob or cell
                #roim = RoiManager(True)
                # Create a ParticleAnalyzer
                measurements = Measurements.AREA + Measurements.MEAN + Measurements.STD_DEV + Measurements.MIN_MAX + Measurements.CENTROID + Measurements.RECT + Measurements.ELLIPSE
                paFlags = ParticleAnalyzer.IN_SITU_SHOW | ParticleAnalyzer.SHOW_MASKS | ParticleAnalyzer.CLEAR_WORKSHEET
                pa = ParticleAnalyzer(paFlags, measurements, table, minSize, Double.POSITIVE_INFINITY, minCircularity, 1.0)

                #pa.setHideOutputImage(True)
                
                # The Result image is copied when CurrIP can still have calibration from loading
                # We want the output to be in terms of pixels, for ease of use, so adjust calibration
                resultsImage.setCalibration(currIP.getCalibration())
                Analyzer.setRedirectImage(resultsImage)
                if not pa.analyze(currIP):
                    print "There was a problem in analyzing", currIP
        
                endTime = time.time()
                if not 'pa' in times:
                    times['pa'] = []
                times['pa'].append(endTime-startTime)
                #for i in range(0, roim.getCount()) :
                #    r = roim.getRoi(i);
                #    r.setColor(Color.red)
                #    r.setStrokeWidth(2)
                
                # The measured areas are listed in the first column of the results table, as a float array:
                newAreas = []
                maxArea = 0
                if table.getColumn(ResultsTable.AREA):
                    for pixArea in table.getColumn(ResultsTable.AREA):
                        a = pixArea * cfg.getValue(ELMConfig.pixelHeight) * cfg.getValue(ELMConfig.pixelWidth)
                        newAreas.append(a)
                        if (a > maxArea):
                            maxArea = a
                        
                # Threshold areas
                idxToRemove = set()
                if cfg.hasValue(ELMConfig.areaMaxPercentThreshold):
                    areaPercentThresh = cfg.getValue(ELMConfig.areaMaxPercentThreshold)
                    for i in range(0,len(newAreas)):
                        if newAreas[i] < (areaPercentThresh * maxArea):
                            idxToRemove.add(i)
                if cfg.hasValue(ELMConfig.areaAbsoluteThreshold):
                    areaAbsoluteThresh = cfg.getValue(ELMConfig.areaAbsoluteThreshold)
                    for i in range(0,len(newAreas)):
                        if newAreas[i] < areaAbsoluteThresh:
                            idxToRemove.add(i)

                for i in sorted(idxToRemove, reverse=True):
                    del newAreas[i]
                
                stats[c][z][t][ELMConfig.UM_AREA] = newAreas
                centroidX = []
                centroidY = []
                roiX = []
                roiY = []
                roiWidth = []
                roiHeight = []
                rArea = []
                # Store all of the other data
                for col in range(0,table.getLastColumn()):
                    newData = table.getColumn(col)
                    if not newData is None:
                        if col == ResultsTable.X_CENTROID:
                            for idx in idxToRemove:
                                centroidX.append(newData[idx])
                        if col == ResultsTable.Y_CENTROID:
                            for idx in idxToRemove:
                                centroidY.append(newData[idx])
                        if col == ResultsTable.ROI_X:
                            for idx in idxToRemove:
                                roiX.append(int(newData[idx]))
                        if col == ResultsTable.ROI_Y:
                            for idx in idxToRemove:
                                roiY.append(int(newData[idx]))
                        if col == ResultsTable.ROI_WIDTH:
                            for idx in idxToRemove:
                                roiWidth.append(int(newData[idx]))
                        if col == ResultsTable.ROI_HEIGHT:
                            for idx in idxToRemove:
                                roiHeight.append(int(newData[idx]))
                        if col == ResultsTable.AREA:
                            for idx in idxToRemove:
                                rArea.append(newData[idx])
                        
                        for i in sorted(idxToRemove, reverse=True):
                            del newData[i]
                    stats[c][z][t][table.getColumnHeading(col)] = newData

                IJ.saveAs('png', os.path.join(outputPath, "PreFiltered_Segmentation_" + dbgOutDesc + "_particles.png"))

                # Remove the segmentation masks for the objects removed
                currProcessor = currIP.getProcessor()
                ff = FloodFiller(currProcessor)
                currIP.getProcessor().setValue(0)
                calib = resultsImage.getCalibration()
                sortedAreaIndices = [i[0] for i in sorted(enumerate(rArea), key=lambda x:x[1])]
                for idx in range(0, len(sortedAreaIndices)):
                    i = sortedAreaIndices[idx]
                    centX = int(calib.getRawX(centroidX[i]))
                    centY = int(calib.getRawY(centroidY[i]))

                    # Since the centroid isn't guaranteed to be part of the blob
                    # search around until an active pixel is found
                    found = False
                    halfWidth = min([roiHeight[i], roiWidth[i]])
                    for offset in range(0,halfWidth):
                        if found:
                            break
                        for x in range(centX-offset,centX+offset+1):
                            if found:
                                break
                            for y in range(centY-offset,centY+offset+1):
                                if not currProcessor.getPixel(x,y) == 0x0:
                                    found = True
                                    finalX = x
                                    finalY = y
                                    break
                    if not found:
                        print "\t\tZ = " + str(z) + ", T = " + str(t) +  ", chan " + chanName + ": ERROR: Never found active pixel for filtered blob, centroid: " + str(centX) + ", " + str(centY)
                    else:
                        currProcessor.setRoi(roiX[i], roiY[i], roiWidth[i], roiHeight[i])
                        ff.fill8(finalX,finalY)
                        #IJ.saveAs('png', os.path.join(outputPath, "Segmentation_" + dbgOutDesc + "_" + str(idx) + ".png"))
                    
                    
                #outImg = pa.getOutputImage()
                IJ.saveAs('png', os.path.join(outputPath, "Segmentation_" + dbgOutDesc + "_particles.png"))
                
                if cfg.hasValue(ELMConfig.createSegMask) and cfg.getValue(ELMConfig.createSegMask) == True:
                    # Create segmentation mask
                    segMask = currIP.duplicate()
                    segMask.setTitle("SegMask_" + dbgOutDesc)
                    # Iterate by smallest area first
                    #  We are more likely to correctly label small areas
                    if len(newAreas) > 0:
                        segProcessor = segMask.getProcessor()
                        if (len(newAreas) > 255):
                            segProcessor = segProcessor.convertToShort(True)
                            segMask.setProcessor(segProcessor)
                        ff = FloodFiller(segProcessor)
                        sortedAreaIndices = [i[0] for i in sorted(enumerate(stats[c][z][t]['Area']), key=lambda x:x[1])]
                        for idx in range(0, len(sortedAreaIndices)):
                            row = sortedAreaIndices[idx]
                            centX = int(stats[c][z][t]['X'][row])
                            centY = int(stats[c][z][t]['Y'][row])
                            roiX = int(stats[c][z][t]['BX'][row])
                            roiY = int(stats[c][z][t]['BY'][row])
                            roiWidth = int(stats[c][z][t]['Width'][row])
                            roiHeight = int(stats[c][z][t]['Height'][row])
                            # Since the centroid isn't guaranteed to be part of the blob
                            # search around until an active pixel is found
                            found = False
                            for xOffset in range(0,roiWidth/2):
                                if found:
                                    break
                                for yOffset in range(0, roiHeight/2):
                                    if found:
                                        break
                                    for x in range(centX-xOffset,centX+xOffset+1):
                                        if found:
                                            break
                                        for y in range(centY-yOffset,centY+yOffset+1):
                                            # original image and this image for masked pixel
                                            # By checking original image, we avoid confusion with a label of 255
                                            if segProcessor.getPixel(x,y) == 255 and currProcessor.getPixel(x,y) == 255:
                                                found = True
                                                finalX = x
                                                finalY = y
                                                break
                            if not found:
                                print "\t\tZ = " + str(z) + ", T = " + str(t) +  ", chan " + chanName + ": ERROR: Never found active pixel for seg mask, centroid: " + str(centX) + ", " + str(centY)
                            else:
                                segProcessor.setRoi(roiX, roiY, roiWidth, roiHeight)
                                segProcessor.setColor(row + 1)
                                ff.fill8(finalX,finalY)
                    
                    lut = LutLoader.openLut(cfg.getValue(ELMConfig.lutPath))
                    segMask.setLut(lut)
                    WindowManager.setTempCurrentImage(segMask);
                    IJ.saveAs('png', os.path.join(outputPath, "SegMask_" + dbgOutDesc + "_particles.png"))
                

                startTime = time.time()

                width = currIP.getWidth();
                height = currIP.getHeight();
                overlayImage = resultsImage.duplicate()
                overlayImage.setTitle("Overlay_" + dbgOutDesc + "_particles")
                if not overlayImage.getType() == ImagePlus.COLOR_RGB:
                    imgConvert = ImageConverter(overlayImage)
                    imgConvert.convertToRGB() 
                overlayProcessor = overlayImage.getProcessor()
                currProcessor = currIP.getProcessor()

                if (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.BRIGHTFIELD):
                    maskColor = 0x0000ff00
                elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.YELLOW):
                    maskColor = 0x000000ff
                elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.RED):
                    maskColor = 0x0000ff00
                elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.GREEN):
                    maskColor = 0x00ff0000
                elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.BLUE):
                    maskColor = 0x00ffff00

                for x in range(0, width):
                    for y in range(0,height):
                        currPix = currProcessor.getPixel(x,y);
                        if not currPix == 0x00000000:
                            overlayProcessor.putPixel(x, y, maskColor)
                            
                endTime = time.time()
                if not 'overlay' in times:
                    times['overlay'] = []
                times['overlay'].append(endTime-startTime)
                
                WindowManager.setTempCurrentImage(overlayImage);
                IJ.saveAs('png', os.path.join(outputPath, "Overlay_" + dbgOutDesc + "_particles.png"))

                #currIP.hide()
                currIP.close()
                resultsImage.close()

    timesAvg = {}
    for key in times:
        timeList = times[key]
        timesAvg[key] = sum(timeList) / len(timeList);
    print("processImage times " + str(timesAvg))
    return stats



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

print("Processed all images in " + str((end - start) / 60) + "m ("  + str(end - start) + "s)")