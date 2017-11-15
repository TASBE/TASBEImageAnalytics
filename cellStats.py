from ij import IJ, ImagePlus, VirtualStack
from ij.process import ImageConverter, ImageProcessor, ByteProcessor
from ij.measure import ResultsTable
from ij.plugin import ChannelSplitter
from ij.plugin.frame import RoiManager
from ij.plugin.filter import ParticleAnalyzer
from ij.measure import Measurements
from ij.gui import Roi

from java.lang import Double
import os, glob, re, time
from jarray import zeros

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
    
####
#
#
####
def main():
    # Input Params
    # TODO: should find a way to input besides hardcoding
    global inputDir;
    global outputDir;
    global numChannels;
    global numZ;
    global noZInFile;
    global chanLabel;

    #inputDir = '/home/nwalczak/Resilio Sync/Resilio/polka_dots_repeat/plate1_not_overlayed'
    #outputDir = '/home/nwalczak/workspace/elm/tmp/test_output_plate1'
    inputDir = '/home/nwalczak/Resilio Sync/Resilio/polka_dots_repeat/plate3_non_overlayed'
    outputDir = '/home/nwalczak/workspace/elm/tmp/test_output_plate3'
    numChannels = 4;
    numZ = 1;
    noZInFile = True;
    chanLabel = ['skip', 'brightfield', 'yellow', 'blue'];
    
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
    
    dsNames = []
    for filePath in imgFiles:
        toks = os.path.basename(filePath).split("_")
        dsNames.append(toks[2])
        
    uniqueNames = list(set(dsNames))
    sort_nicely(uniqueNames)
    
    dsResults = []
    for datasetName in uniqueNames:
        dsImgFiles = [];
        for i in range(0, len(imgFiles)) :
            if datasetName == dsNames[i] :
                dsImgFiles.append(imgFiles[i])

        datasetPath = os.path.join(outputDir, datasetName)
        if not os.path.exists(datasetPath):
            os.mkdir(datasetPath)
            
        start = time.time()
        dsResults.append(datasetName + ", " + processDataset(datasetName, dsImgFiles))
        end = time.time()
        print("Processed datset " + datasetName + " in " + str(end - start) + " s")
        
    resultsFile = open(os.path.join(outputDir, "AllResults.csv"), "w")
    resultsFile.write("dataset, frame, brightfield area, yellow area, blue area, percent yellow, percent blue, classification \n")
    for result in dsResults:
        resultsFile.write(result);
    resultsFile.close()    

####



####
#
#
####
def processDataset(datasetName, imgFiles):
    global inputDir;
    global outputDir;
    global numChannels;
    global numZ;
    global noZInFile;
    global chanLabel;
    
    datasetPath = os.path.join(outputDir, datasetName)
    
    firstImage = IJ.openImage(imgFiles[0]);
    imgWidth = firstImage.getWidth();
    imgHeight = firstImage.getHeight();

    # Count how many images we have for each channel/Z slice
    imgFileCats = [[[] for z in range(numZ)] for c in range(numChannels)]
    for c in range(0, numChannels):
        chanStr = 'ch%(channel)02d' % {"channel" : c};
        for z in range(0, numZ):
            zStr =  'z%(depth)02d' % {"depth" : z};
            for imgPath in imgFiles:
                fileName = os.path.basename(imgPath)
                if chanStr in fileName and (noZInFile or zStr in fileName):
                    imgFileCats[c][z].append(fileName)
    
    # Load all images
    images = [[0 for z in range(numZ)] for c in range(numChannels)]
    for c in range(0, numChannels):
        for z in range(0, numZ):
            if not imgFileCats[c][z]:
                continue;
            
            imSeq = VirtualStack(imgWidth, imgHeight, firstImage.getProcessor().getColorModel(), inputDir)
            for fileName in imgFileCats[c][z]:
                imSeq.addSlice(fileName);
            images[c][z] = ImagePlus()
            images[c][z].setStack(imSeq)
            images[c][z].setTitle(datasetName + ", channel " + str(c) + ", z " + str(z))
    
    # Process images
    # We need to avoid the scale bar in the bottom of the image, so set a roi that doesn't include it
    analysisRoi = Roi(0,0,512,480)
    areas = []
    for c in range(0, numChannels):
        chanStr = 'ch%(channel)02d' % {"channel" : c};
        for z in range(0, numZ):
            zStr =  'z%(depth)02d' % {"depth" : z};
            currIP = images[c][z];
            #currIP.show()
            # We need to get to a grayscale image, which will be done differently for different channels
            if (chanLabel[c] == "brightfield"):
                toGray = ImageConverter(currIP)
                toGray.convertToGray8()
                minCircularity = 0.02 # We want to identify one big cell ball, so ignore small less circular objects
                minSize = 40
                darkBackground = False
            elif (chanLabel[c] == "blue"): # 
                imgChanns = ChannelSplitter.split(currIP);
                currIP = imgChanns[2];
                minCircularity = 0.02
                minSize = 5
                darkBackground = True
            elif (chanLabel[c] == "yellow"):
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
            elif (chanLabel[c] == "skip"):
                areas.append([])
                continue 
            currIP.show()
            currIP.getProcessor().setAutoThreshold("Default", darkBackground, ImageProcessor.NO_LUT_UPDATE)
            threshRange = currIP.getProcessor().getMaxThreshold() - currIP.getProcessor().getMinThreshold()
            print "\tChannel %14s threshold:\t [%d, %d]\t range: %d" % (chanLabel[c],currIP.getProcessor().getMinThreshold(), currIP.getProcessor().getMaxThreshold(), threshRange)
            if currIP.getType() != ImagePlus.GRAY8 :
                print "\tChannel " + chanLabel[c] + " is not GRAY8, instead type is %d" % currIP.getType()
            if threshRange > 230:
                print "\t\tIgnored Objects due to threshold range!"
                areas.append([])
                continue 
            IJ.run(currIP, "Convert to Mask", "")
            IJ.run(currIP, "Close-", "")
            currIP.setRoi(analysisRoi)
            
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
            paFlags = ParticleAnalyzer.IN_SITU_SHOW | ParticleAnalyzer.SHOW_OUTLINES | ParticleAnalyzer.ADD_TO_MANAGER | ParticleAnalyzer.EXCLUDE_EDGE_PARTICLES | ParticleAnalyzer.SHOW_ROI_MASKS | ParticleAnalyzer.CLEAR_WORKSHEET
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
            currIP.hide()
        
    resultsFile = open(os.path.join(datasetPath, datasetName + "_results.txt"), "w")
    resultsFile.write("frame, brightfield area, yellow area, blue area, percent yellow, percent blue, classification \n")
    for c in range(0, numChannels) :
        chanStr = '_ch%(channel)02d_' % {"channel" : c};
        area = 0;
        writeArea = False
        if (chanLabel[c] == "brightfield"):
            if not areas[c] :
                area = 0;
            else:
                area = max(areas[c])
                writeArea = True
            totalArea = area
           
        elif (chanLabel[c] == "blue"): #
            if not areas[c] :
                area = 0;
            else:
                area = sum(areas[c]) 
                writeArea = True
            blueArea = area
        elif (chanLabel[c] == "yellow"):
            if not areas[c] :
                area = 0;
            else:
                area = sum(areas[c])
                writeArea = True
            yellowArea = area
        elif (chanLabel[c] == "skip"):
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

####
#
#
####        
if __name__ == "__main__":
    start = time.time()
    main()
    end = time.time()
    print("Processed all images in " + str(end - start) + " s")