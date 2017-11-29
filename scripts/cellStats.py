from ij import IJ, ImagePlus, VirtualStack, WindowManager
from ij.process import ImageConverter, ImageProcessor, ByteProcessor
from ij.measure import ResultsTable
from ij.plugin import ChannelSplitter
#from ij.plugin.frame import RoiManager
from ij.plugin.filter import ParticleAnalyzer
from ij.measure import Measurements
from ij.gui import Roi

from java.lang import Double
import os, glob, re, time, sys
import ConfigParser
import io
#from jarray import zeros

#
# Stackoverflow code for numerically sorting strings
# https://stackoverflow.com/questions/4623446/how-do-you-sort-files-numerically
#
def tryint(s):
    try:
        return int(s)
    except:
        return s

def alphanum_key(s):
    """ Turn a string into a list of string and number chunks.
        "z23a" -> ["z", 23, "a"]
    """
    return [ tryint(c) for c in re.split('([0-9]+)', s) ]

def sort_nicely(l):
    """ Sort the given list in the way that humans expect.
    """
    l.sort(key=alphanum_key)
    
#
#
#
def printUsage():
    global numChannels;
    global numZ;
    
    print "This script will read tif files from an input directory and compute statistics on the cell images."
    print "The script must be pointed to a configuration ini file that will define several important aspects."
    print "The following parameters are recognized in the [Config] section:"

    print "inputDir  - Dir to read in tif files from. (Required)"
    print "outputDir - Dir to write output to. (Required)"
    print "numChannels - Number of channels in dataset, defaults to 4"
    print "numZ - Number of Z slices in dataset, defaults to 1"
    print "noZInFile - True if the z slice does not appear in the filename, otherwise false"
    print "chanLabel - Label that describes source for each channel, default skip, yellow, blue, brightfield"
    print "analysisRoi - Rectangular area to perform cell detection on, default 0,0,512,480"
    print "dsNameIdx - Index of well name within filename, when delimiting on underscores (_)"

    print "Usage: "
    print "<cfgPath>"
    
####
#
#
####
def main(cfg):
    # Input Params
    # TODO: should find a way to input besides hardcoding

    print "Processing input dir " + cfg.inputDir;
    print "Outputting in " + cfg.outputDir;

    # Get all images in the input dir
    imgFiles = glob.glob(os.path.join(cfg.inputDir, "*.tif"))
    # Ensure we have tifs
    if (len(imgFiles) < 1):
        print "No tif files found in input directory!  Input dir: " + cfg.inputDir
        quit()

    # Sort filenames so they are in order by z and ch
    sort_nicely(imgFiles)

    dsNames = []
    for filePath in imgFiles:
        toks = os.path.basename(filePath).split("_")
        dsNames.append(toks[cfg.dsNameIdx])

    uniqueNames = list(set(dsNames))
    sort_nicely(uniqueNames)

    dsResults = []
    for datasetName in uniqueNames:
        dsImgFiles = [];
        for i in range(0, len(imgFiles)) :
            if datasetName == dsNames[i] :
                dsImgFiles.append(imgFiles[i])

        datasetPath = os.path.join(cfg.outputDir, datasetName)
        if not os.path.exists(datasetPath):
            os.makedirs(datasetPath)
            
        start = time.time()
        dsResults.append(datasetName + ", " + processDataset(cfg, datasetName, dsImgFiles))
        end = time.time()
        print("Processed datset " + datasetName + " in " + str(end - start) + " s")

    resultsFile = open(os.path.join(cfg.outputDir, "AllResults.csv"), "w")
    resultsFile.write("dataset, frame, brightfield area, yellow area, blue area, percent yellow, percent blue, classification \n")
    for result in dsResults:
        resultsFile.write(result);
    resultsFile.close()    

####



####
#
#
####
def processDataset(cfg, datasetName, imgFiles):
    datasetPath = os.path.join(cfg.outputDir, datasetName)
    
    firstImage = IJ.openImage(imgFiles[0]);
    imgWidth = firstImage.getWidth();
    imgHeight = firstImage.getHeight();

    # Count how many images we have for each channel/Z slice
    imgFileCats = [[[] for z in range(cfg.numZ)] for c in range(cfg.numChannels)]
    for c in range(0, cfg.numChannels):
        chanStr = 'ch%(channel)02d' % {"channel" : c};
        for z in range(0, cfg.numZ):
            zStr =  'z%(depth)02d' % {"depth" : z};
            for imgPath in imgFiles:
                fileName = os.path.basename(imgPath)
                if chanStr in fileName and (cfg.noZInFile or zStr in fileName):
                    imgFileCats[c][z].append(fileName)
    
    # Load all images
    images = [[0 for z in range(cfg.numZ)] for c in range(cfg.numChannels)]
    for c in range(0, cfg.numChannels):
        for z in range(0, cfg.numZ):
            if not imgFileCats[c][z]:
                continue;
            
            imSeq = VirtualStack(imgWidth, imgHeight, firstImage.getProcessor().getColorModel(), cfg.inputDir)
            for fileName in imgFileCats[c][z]:
                imSeq.addSlice(fileName);
            images[c][z] = ImagePlus()
            images[c][z].setStack(imSeq)
            images[c][z].setTitle(datasetName + ", channel " + str(c) + ", z " + str(z))
    
    # Process images
    areas = []
    for c in range(0, cfg.numChannels):
        chanStr = 'ch%(channel)02d' % {"channel" : c};
        for z in range(0, cfg.numZ):
            zStr =  'z%(depth)02d' % {"depth" : z};
            currIP = images[c][z];
            #currIP.show()
            # We need to get to a grayscale image, which will be done differently for different channels
            if (cfg.chanLabel[c] == "brightfield"):
                toGray = ImageConverter(currIP)
                toGray.convertToGray8()
                minCircularity = 0.02 # We want to identify one big cell ball, so ignore small less circular objects
                minSize = 40
                darkBackground = False
            elif (cfg.chanLabel[c] == "blue"): # 
                imgChanns = ChannelSplitter.split(currIP);
                currIP = imgChanns[2];
                minCircularity = 0.02
                minSize = 5
                darkBackground = True
            elif (cfg.chanLabel[c] == "yellow"):
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
                minCircularity = 0.02
                minSize = 5
                darkBackground = True
            elif (cfg.chanLabel[c] == "skip"):
                areas.append([])
                continue 
            WindowManager.setTempCurrentImage(currIP);
            #currIP.show()
            currIP.getProcessor().setAutoThreshold("Default", darkBackground, ImageProcessor.NO_LUT_UPDATE)
            threshRange = currIP.getProcessor().getMaxThreshold() - currIP.getProcessor().getMinThreshold()
            print "\tChannel %14s threshold:\t [%d, %d]\t range: %d" % (cfg.chanLabel[c],currIP.getProcessor().getMinThreshold(), currIP.getProcessor().getMaxThreshold(), threshRange)
            if currIP.getType() != ImagePlus.GRAY8 :
                print "\tChannel " + cfg.chanLabel[c] + " is not GRAY8, instead type is %d" % currIP.getType()
            if threshRange > 230:
                print "\t\tIgnored Objects due to threshold range!"
                areas.append([])
                continue 
            IJ.run(currIP, "Convert to Mask", "")
            IJ.run(currIP, "Close-", "")
            currIP.setRoi(cfg.analysisRoi)
            
            # Create a table to store the results
            table = ResultsTable()
            # Create a hidden ROI manager, to store a ROI for each blob or cell
            #roim = RoiManager(True)
            # Create a ParticleAnalyzer
            paFlags = ParticleAnalyzer.IN_SITU_SHOW | ParticleAnalyzer.SHOW_OUTLINES | ParticleAnalyzer.EXCLUDE_EDGE_PARTICLES | ParticleAnalyzer.SHOW_ROI_MASKS | ParticleAnalyzer.CLEAR_WORKSHEET
            pa = ParticleAnalyzer(paFlags, Measurements.AREA, table, minSize, Double.POSITIVE_INFINITY, minCircularity, 1.0)
            #pa.setHideOutputImage(True)
    
            if not pa.analyze(currIP):
                print "There was a problem in analyzing", currIP
    
            #for i in range(0, roim.getCount()) :
            #    r = roim.getRoi(i);
            #    r.setColor(Color.red)
            #    r.setStrokeWidth(2)
            
            #outImg = pa.getOutputImage()
            IJ.saveAs('png', os.path.join(datasetPath, "Segmentation_" + datasetName + "_" + zStr + "_" + chanStr + "_particles.png"))
    
            # The measured areas are listed in the first column of the results table, as a float array:
            areas.append(table.getColumn(0))
            #currIP.hide()
        
    resultsFile = open(os.path.join(datasetPath, datasetName + "_results.txt"), "w")
    resultsFile.write("frame, brightfield area, yellow area, blue area, percent yellow, percent blue, classification \n")
    for c in range(0, cfg.numChannels) :
        chanStr = '_ch%(channel)02d_' % {"channel" : c};
        area = 0;
        writeArea = False
        if (cfg.chanLabel[c] == "brightfield"):
            if not areas[c] :
                area = 0;
            else:
                area = max(areas[c])
                writeArea = True
            totalArea = area
           
        elif (cfg.chanLabel[c] == "blue"): #
            if not areas[c] :
                area = 0;
            else:
                area = sum(areas[c]) 
                writeArea = True
            blueArea = area
        elif (cfg.chanLabel[c] == "yellow"):
            if not areas[c] :
                area = 0;
            else:
                area = sum(areas[c])
                writeArea = True
            yellowArea = area
        elif (cfg.chanLabel[c] == "skip"):
            continue
        if writeArea:
            chanResultsFile = open(os.path.join(datasetPath, datasetName + chanStr + "areas.txt"), "w")
            for area in areas[c] :
                chanResultsFile.write("%d\n" % area)
            chanResultsFile.close()  
    if totalArea == 0:
        percentBlue = 0
        percentYellow = 0
    else:  
        percentBlue = blueArea / totalArea
        percentYellow = yellowArea / totalArea
    resultsString = "%d,\t\t\t %d,\t\t %d,\t\t %d,\t\t %0.4f,\t\t %0.4f \n" % (1, totalArea, yellowArea, blueArea, percentYellow, percentBlue)
    resultsFile.write(resultsString)
    resultsFile.close()
    return resultsString

class config:
    numChannels = 4;
    numZ = 1;
    noZInFile = True;
    #chanLabel = ['skip', 'brightfield', 'yellow', 'blue'];
    chanLabel = ['skip', 'yellow', 'blue', 'brightfield'];
    inputDir = '';
    outputDir = '';
    # We need to avoid the scale bar in the bottom of the image, so set a roi that doesn't include it
    #analysisRoi = Roi(0,0,512,480)
    analysisRoi = Roi(0,0,1024,980)
    # Determines what index the dataset name is within the tokenized filename
    dsNameIdx = 4;
    
    
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
cfgParser = ConfigParser.RawConfigParser(allow_no_value=True)
rv = cfgParser.readfp(open(cfgPath))

if not cfgParser.has_section("Config"):
    print "Config file doesn't contain [Config] section! Path: " + cfgPath;
    quit(1)

cfg = config()
if cfgParser.has_option("Config", "numchannels") :
    numChannels = int(cfgParser.get("Config", "numchannels"))
else:
    numChannels = cfg.numChannels
for option in cfgParser.options("Config"):
    if option == "inputdir":
        cfg.inputDir = cfgParser.get("Config", option)
    elif option == "outputdir":
        cfg.outputDir = cfgParser.get("Config", option)
    elif option == "numchannels":
        cfg.numChannels = int(cfgParser.get("Config", option))
    elif option == "numz":
        cfg.numZ = int(cfgParser.get("Config", option))
    elif option == "nozinfile":
        cfg.noZInFile  = bool(cfgParser.get("Config", option))
    elif option == "chanlabel":
        toks = cfgParser.get("Config", option).split(",")
        if not len(toks) == numChannels:
            print "Improper value for chanLabel config, expected " + numChannels + " comma separated values!  Received " + str(len(toks))
        cfg.chanLabel = []
        for tok in toks:
            cfg.chanLabel.append(tok.strip())
    elif option == "analysisroi":
        toks = cfgParser.get("Config", option).split(",")
        if not len(toks) == 4:
            print "Improper value for analysisRoi config, expected " + 4+ " comma separated values!  Received " + str(len(toks))
        cfg.analysisRoi = Roi(int(toks[0]),int(toks[1]),int(toks[2]),int(toks[3]))
    elif option == "dsnameidx":
        cfg.dsNameIdx = int(cfgParser.get("Config", option))
    else:
        print "Warning, unrecognized config option: " + option

# Print Config
print("Using Config:")
print("\tinputDir:\t"    + cfg.inputDir)
print("\toutputDir:\t"   + cfg.outputDir)
print("\tnumChannels:\t" + str(cfg.numChannels))
print("\tnumZ:\t\t"        + str(cfg.numZ))
print("\tnoZInFile:\t"   + str(cfg.noZInFile))
print("\tchanLabel:\t"   + ", ".join(cfg.chanLabel))
print("\tanalysisRoi:\t" + str(cfg.analysisRoi))
print("\n")


    
start = time.time()
main(cfg)
end = time.time()
print("Processed all images in " + str(end - start) + " s")