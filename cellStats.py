from ij import IJ, ImagePlus, VirtualStack, Menus
from ij.process import ImageConverter, AutoThresholder, ImageProcessor
from ij.measure import ResultsTable
from ij.plugin import ChannelSplitter
from ij.plugin.frame import RoiManager
from ij.plugin.filter import ParticleAnalyzer
from ij.measure import Measurements
from ij.gui import Roi

from java.lang import Double
from java.awt import Color

import os, glob, re


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
    

# Input Params
# TODO: should find a way to input besides hardcoding
datasetName = 'A2'
inputDir = '/home/nwalczak/workspace/elm/tmp/A2'
outputDir = '/home/nwalczak/workspace/elm/tmp/test_output'
numChannels = 3;
numZ = 1;
noZInFile = True;

chanLabel = ['brightfield', 'yellow', 'blue'];

# Get currently selected image
#imp = WindowManager.getCurrentImage()
#imp = IJ.openImage('http://fiji.sc/samples/FakeTracks.tif')
#fo = FolderOpener()


imgFiles = glob.glob(os.path.join(inputDir, "*.tif"))
# Ensure we have tifs
if (len(imgFiles) < 1):
    print "No tif files found in input directory!  Input dir: " + inputDir
    quit()

sort_nicely(imgFiles)
# Get info about image dimensions - needed for creating stacks
firstImage = IJ.openImage(imgFiles[0]);
imgWidth = firstImage.getWidth();
imgHeight = firstImage.getHeight();

# Count how many images we have for each channel/Z slice
imgFileCats = [[[] for z in range(numZ)] for c in range(numChannels)]
for c in range(0, numChannels):
    chanStr = 'ch%(channel)02d' % {"channel" : c + 1};
    for z in range(0, numZ):
        zStr =  'z%(depth)02d' % {"depth" : z};
        count = 0;
        for imgPath in imgFiles:
            fileName = os.path.basename(imgPath)
            if chanStr in fileName and (noZInFile or zStr in fileName):
                imgFileCats[c][z].append(fileName)

# Load all images
currZ = 0;images = [[0 for z in range(numZ)] for c in range(numChannels)]
for c in range(0, numChannels):
    for z in range(0, numZ):
        imSeq = VirtualStack(imgWidth, imgHeight, firstImage.getProcessor().getColorModel(), inputDir)
        for fileName in imgFileCats[c][z]:
            imSeq.addSlice(fileName);
        images[c][z] = ImagePlus()
        images[c][z].setStack(imSeq)
        images[c][z].setTitle(datasetName + ", channel " + str(c) + ", z " + str(z))

# Process images
thresholder = AutoThresholder();
# We need to avoid the scale bar in the bottom of the image, so set a roi that doesn't include it
analysisRoi = Roi(0,0,512,480)
areas = []
for c in range(0, numChannels):
    chanStr = 'ch%(channel)02d' % {"channel" : c + 1};
    for z in range(0, numZ):
        zStr =  'z%(depth)02d' % {"depth" : z};
        currIP = images[c][z];
        currIP.setRoi(analysisRoi)
        currIP.show()
        # We need to get to a grayscale image, which will be done differently for different channels
        if (chanLabel[c] == "brightfield"):
            toGray = ImageConverter(currIP)
            toGray.convertToGray8()
            minCircularity = 0.2 # We want to identify one big cell ball, so ignore small less circular objects
            minSize = 40
        elif (chanLabel[c] == "blue"): # 
            imgChanns = ChannelSplitter.split(currIP);
            currIp = imgChanns[2];
            minCircularity = 0.02
            minSize = 5
        elif (chanLabel[c] == "yellow"):
            imgChanns = ChannelSplitter.split(currIP);
            currIp = imgChanns[1];
            minCircularity = 0.02
            minSize = 5
        currIP.getProcessor().setAutoThreshold("Default", False, ImageProcessor.NO_LUT_UPDATE)
        IJ.run(currIP, "Convert to Mask", "")
        IJ.run(currIP, "Close-", "")
        currIP.show()
        
        # Create a table to store the results
        table = ResultsTable()
        # Create a hidden ROI manager, to store a ROI for each blob or cell
        roim = RoiManager(True)
        # Create a ParticleAnalyzer, with arguments:
        # 1. options (could be SHOW_ROI_MASKS, SHOW_OUTLINES, SHOW_MASKS, SHOW_NONE, ADD_TO_MANAGER, and others; combined with bitwise-or)
        # 2. measurement options (see [http://imagej.net/developer/api/ij/measure/Measurements.html Measurements])
        # 3. a ResultsTable to store the measurements
        # 4. The minimum size of a particle to consider for measurement
        # 5. The maximum size (idem)
        # 6. The minimum circularity of a particle
        # 7. The maximum circularity
        paFlags = ParticleAnalyzer.IN_SITU_SHOW | ParticleAnalyzer.SHOW_OUTLINES | ParticleAnalyzer.ADD_TO_MANAGER | ParticleAnalyzer.INCLUDE_HOLES | ParticleAnalyzer.EXCLUDE_EDGE_PARTICLES | ParticleAnalyzer.SHOW_ROI_MASKS
        pa = ParticleAnalyzer(paFlags, Measurements.AREA, table, minSize, Double.POSITIVE_INFINITY, minCircularity, 1.0)
        #pa.setHideOutputImage(True)

        if pa.analyze(currIP):
            print "All ok"
        else:
            print "There was a problem in analyzing", currIP

        for i in range(0, roim.getCount()) :
            r = roim.getRoi(i);
            r.setColor(Color.red)
            r.setStrokeWidth(2)
        
        outImg = pa.getOutputImage()
        IJ.saveAs('png', os.path.join(outputDir, "Segmentation_" + datasetName + "_" + zStr + "_" + chanStr + "_particles.png"))

        # The measured areas are listed in the first column of the results table, as a float array:
        areas.append(table.getColumn(0))
    
resultsFile = open(os.path.join(outputDir, datasetName + "results.txt"), "w")
for c in range(0, numChannels) :
    area = 0;
    if (chanLabel[c] == "brightfield"):
        area = max(areas[c])
        totalArea = area
    elif (chanLabel[c] == "blue"): # 
        area = sum(areas[c])
        blueArea = area
    elif (chanLabel[c] == "yellow"):
        area = sum(areas[c])
        yellowArea = area
    resultsFile.write("%s area: %d \n" % (chanLabel[c], area))
percentBlue = blueArea / totalArea
percentyellow = yellowArea / totalArea
resultsFile.write("Percent blue area: %0.4f \n" % percentBlue)
resultsFile.write("Percent yellow area: %0.4f \n" % percentyellow)
resultsFile.close()

cmds = Menus.getCommands()
tmp = 5;

        
        