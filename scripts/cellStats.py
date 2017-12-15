from ij import IJ, ImagePlus, VirtualStack, WindowManager
from ij.process import ImageConverter, ImageProcessor, ByteProcessor, ColorProcessor
from ij.measure import ResultsTable
from ij.plugin import ChannelSplitter, ImageCalculator
from ij.gui import Roi
#from ij.plugin.frame import RoiManager
from ij.plugin.filter import ParticleAnalyzer, Analyzer
from ij.measure import Measurements


from java.lang import Double
import os, glob, re, time, sys



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
def getCSVHeader(cfg):
    outputChans = [];
    for chan in cfg.getValue(ELMConfig.chanLabel):
        if not chan == ELMConfig.SKIP and not chan == ELMConfig.BRIGHTFIELD:
            outputChans.append(chan)
    headerString = "well, brightfield area (um^2), "
    for chan in outputChans:
        headerString += chan + " area (um^2), "
        headerString += "percent " + chan + ", " 
    headerString += "classification \n"
    return headerString

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
    wellRE   = re.compile("^[A-Z][0-9]+$")
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
        dsResults.append(wellName + ", " + processDataset(cfg, wellName, dsImgFiles))
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
    
    firstImage = IJ.openImage(imgFiles[0]);
    imgWidth = firstImage.getWidth();
    imgHeight = firstImage.getHeight();

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
                    imgFileCats[c][z].append(fileName)

    # Check for no images
    if not addedImages:
        print "Failed to add any images to chan/z categories! Problem with input dir?"
        quit(1)

    # Load all images
    images = [[0 for z in range(cfg.getValue(ELMConfig.numZ))] for c in range(cfg.getValue(ELMConfig.numChannels))]
    for c in range(0, cfg.getValue(ELMConfig.numChannels)):
        for z in range(0, cfg.getValue(ELMConfig.numZ)):
            if not imgFileCats[c][z]:
                continue;
            
            imSeq = VirtualStack(imgWidth, imgHeight, firstImage.getProcessor().getColorModel(), cfg.getValue(ELMConfig.inputDir))
            for fileName in imgFileCats[c][z]:
                imSeq.addSlice(fileName);
            images[c][z] = ImagePlus()
            images[c][z].setStack(imSeq)
            images[c][z].setTitle(datasetName + ", channel " + str(c) + ", z " + str(z))
    
    # Process images
    stats = processImages(cfg, datasetName, datasetPath, images)

    # Output Results
    resultsFile = open(os.path.join(datasetPath, datasetName + "_results.csv"), "w")
    resultsFile.write(getCSVHeader(cfg));
    channelAreas = dict() 
    for c in range(0, cfg.getValue(ELMConfig.numChannels)) :
        chanStr = '_ch%(channel)02d_' % {"channel" : c};
        area = 0;
        writeStats = False
        # Handle brigthfield channel
        if (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.BRIGHTFIELD):
            if not stats[c][ELMConfig.UM_AREA] :
                area = 0;
            else:
                area = sum(stats[c][ELMConfig.UM_AREA])
                writeStats = True
            channelAreas["totalArea"] = area
        # Handle Fluorscent Channels   
        elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.BLUE) \
                or (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.RED) \
                or (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.GREEN) \
                or (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.YELLOW): #
            if not stats[c][ELMConfig.UM_AREA] :
                area = 0;
            else:
                area = sum(stats[c][ELMConfig.UM_AREA])
                writeStats = True
            channelAreas[cfg.getValue(ELMConfig.chanLabel)[c]] = area
        # Skip channel
        elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.SKIP):
            continue
        # Write out individual areas per channel
        if writeStats:
            chanResultsFile = open(os.path.join(datasetPath, datasetName + chanStr + "stats.csv"), "w")
            numParticles = len(stats[c][ELMConfig.UM_AREA])
            # Writer Header
            keys = sorted(stats[c].keys())
            headerKeys =  [ key.replace("%", "percent ") for key in keys ]
            chanResultsFile.write(", ".join(headerKeys) + "\n")
            for particle in range(0, numParticles):
                line = ""
                for measure in keys:
                    if stats[c][measure]:
                        line += "%10.4f, " % stats[c][measure][particle]
                    else:
                        line += "      N/A, "
                line = line[0:len(line)-2] + "\n"
                chanResultsFile.write(line);
            chanResultsFile.close()

    if channelAreas["totalArea"] == 0:
        channelAreas["totalArea"] = 0.000000001

    outputChans = [];
    for chan in cfg.getValue(ELMConfig.chanLabel):
        if not chan == ELMConfig.SKIP and not chan == ELMConfig.BRIGHTFIELD:
            outputChans.append(chan)

    resultsString = "\t\t\t %10.4f," % (channelAreas["totalArea"])
    numChans = len(outputChans)
    for i in range(0, numChans):
        resultsString += "\t\t %10.4f," % channelAreas[outputChans[i]]
        resultsString += "\t\t %0.4f" % (channelAreas[outputChans[i]] /  channelAreas["totalArea"])
        if (i + 1 < numChans) :
            resultsString += ","
    resultsString += "\n"   
    resultsFile.write(resultsString)
    resultsFile.close()
    return resultsString


####
#
#  All of the processing that happens for each image
#
####
def processImages(cfg, wellName, wellPath, images):
    stats = [dict() for x in range(0, cfg.getValue(ELMConfig.numChannels))]

    for c in range(0, cfg.getValue(ELMConfig.numChannels)):
        chanStr = 'ch%(channel)02d' % {"channel" : c};
        for z in range(0, cfg.getValue(ELMConfig.numZ)):
            zStr = cfg.getZStr(z);
            currIP = images[c][z];
            resultsImage = currIP.duplicate()
            if cfg.getValue(ELMConfig.debugOutput):
                WindowManager.setTempCurrentImage(currIP);
                IJ.saveAs('png', os.path.join(wellPath, "Orig_" + wellName + "_" + zStr + "_" + chanStr + ".png"))
            # We need to get to a grayscale image, which will be done differently for different channels
            if (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.BRIGHTFIELD):
                toGray = ImageConverter(currIP)
                toGray.convertToGray8()
                minCircularity = 0.001 # We want to identify one big cell ball, so ignore small less circular objects
                minSize = 500
                darkBackground = False
            elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.BLUE) \
                    or (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.RED) \
                    or (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.GREEN): #
                chanIdx = 2
                if (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.RED):
                    chanIdx = 0
                elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.GREEN):
                    chanIdx = 1;
                imgChanns = ChannelSplitter.split(currIP);
                currIP = imgChanns[chanIdx];
                minCircularity = 0.001
                minSize = 5
                darkBackground = True
            elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.YELLOW):
                title = currIP.getTitle()
                # Create a new image that consists of the average of the red & green channels
                width = currIP.getWidth();
                height = currIP.getHeight();
                newPix = ByteProcessor(width, height)
                for x in range(0, width) :
                    for y in range(0,height) :
                        currPix = currIP.getPixel(x,y);
                        newPix.putPixel(x, y, (currPix[0] + currPix[1]) / 2)
                
                currIP = ImagePlus(title, newPix)
                minCircularity = 0.001
                minSize = 5
                darkBackground = True
            elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.SKIP):
                continue
            WindowManager.setTempCurrentImage(currIP);

            if cfg.getValue(ELMConfig.debugOutput):
                IJ.saveAs('png', os.path.join(wellPath, "Processing_" + wellName + "_" + zStr + "_" + chanStr + ".png"))\

            upperThreshImg = ImagePlus
            upperThreshImg = currIP.duplicate()

            currIP.getProcessor().setAutoThreshold("Default", darkBackground, ImageProcessor.NO_LUT_UPDATE)
            threshRange = currIP.getProcessor().getMaxThreshold() - currIP.getProcessor().getMinThreshold()
            print "\tChannel %14s threshold:\t [%d, %d]\t range: %d" % (cfg.getValue(ELMConfig.chanLabel)[c],currIP.getProcessor().getMinThreshold(), currIP.getProcessor().getMaxThreshold(), threshRange)
            if currIP.getType() != ImagePlus.GRAY8 :
                print "\tChannel " + cfg.getValue(ELMConfig.chanLabel)[c] + " is not GRAY8, instead type is %d" % currIP.getType()
            if threshRange > 230:
                print "\t\tIgnored Objects due to threshold range!"
                stats[c][ELMConfig.UM_AREA] = []
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
                currIP = compositeMask
                WindowManager.setTempCurrentImage(currIP);                
                
            if cfg.getValue(ELMConfig.debugOutput):
                WindowManager.setTempCurrentImage(currIP);
                IJ.saveAs('png', os.path.join(wellPath, "Binary_" + wellName + "_" + zStr + "_" + chanStr + ".png"))
            currIP.setRoi(Roi(int(cfg.getValue(ELMConfig.analysisRoi)[0]), int(cfg.getValue(ELMConfig.analysisRoi)[1]), int(cfg.getValue(ELMConfig.analysisRoi)[2]), int(cfg.getValue(ELMConfig.analysisRoi)[3])))

            if cfg.getValue(ELMConfig.debugOutput):
                WindowManager.setTempCurrentImage(resultsImage);
                IJ.saveAs('png', os.path.join(wellPath, "resultsImage" + wellName + "_" + zStr + "_" + chanStr + ".png"))
                WindowManager.setTempCurrentImage(currIP);

            # Create a table to store the results
            table = ResultsTable()
            # Create a hidden ROI manager, to store a ROI for each blob or cell
            #roim = RoiManager(True)
            # Create a ParticleAnalyzer
            paFlags = ParticleAnalyzer.IN_SITU_SHOW | ParticleAnalyzer.SHOW_MASKS | ParticleAnalyzer.CLEAR_WORKSHEET
            pa = ParticleAnalyzer(paFlags, Measurements.ALL_STATS, table, minSize, Double.POSITIVE_INFINITY, minCircularity, 1.0)

            #pa.setHideOutputImage(True)

            Analyzer.setRedirectImage(resultsImage)
            if not pa.analyze(currIP):
                print "There was a problem in analyzing", currIP
    
            #for i in range(0, roim.getCount()) :
            #    r = roim.getRoi(i);
            #    r.setColor(Color.red)
            #    r.setStrokeWidth(2)
            
            #outImg = pa.getOutputImage()
            IJ.saveAs('png', os.path.join(wellPath, "Segmentation_" + wellName + "_" + zStr + "_" + chanStr + "_particles.png"))

            width = currIP.getWidth();
            height = currIP.getHeight();
            overlayProcessor = ColorProcessor(width, height)
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

            for x in range(0, width) :
                for y in range(0,height) :
                    currPix = currProcessor.getPixel(x,y);
                    if currPix == 0x00000000:
                        overlayProcessor.putPixel(x, y, resultsImage.getProcessor().getPixel(x,y))
                    else:
                        overlayProcessor.putPixel(x, y, maskColor)
            overlayImage = ImagePlus("Overlay_" + wellName + "_" + zStr + "_" + chanStr + "_particles", overlayProcessor)
            WindowManager.setTempCurrentImage(overlayImage);
            IJ.saveAs('png', os.path.join(wellPath, "Overlay_" + wellName + "_" + zStr + "_" + chanStr + "_particles.png"))

            # The measured areas are listed in the first column of the results table, as a float array:
            newAreas = []
            if table.getColumn(ResultsTable.AREA):
                for pixArea in table.getColumn(ResultsTable.AREA):
                    newAreas.append(pixArea * cfg.getValue(ELMConfig.pixelHeight) * cfg.getValue(ELMConfig.pixelWidth))
            stats[c][ELMConfig.UM_AREA] = newAreas

            # Store all of the other data
            for col in range(0,table.getLastColumn()):
                stats[c][table.getColumnHeading(col)] = table.getColumn(col)

            #currIP.hide()

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

print("Processed all images in " + str(end - start) + " s")