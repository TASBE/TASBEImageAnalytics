# Copyright (C) 2011 - 2019, Raytheon BBN Technologies and contributors listed
# in the AUTHORS file in TASBE Flow Analytics distribution's top directory.
#
# This file is part of the TASBE Flow Analytics package, and is distributed
# under the terms of the GNU General Public License, with a linking
# exception, as described in the file LICENSE in the TASBE Image Analysis
# package distribution's top directory.

from fiji.plugin.trackmate import Model
from fiji.plugin.trackmate import Settings
from fiji.plugin.trackmate import TrackMate
from fiji.plugin.trackmate import SelectionModel
from fiji.plugin.trackmate import Logger
from fiji.plugin.trackmate.detection import LogDetectorFactory
from fiji.plugin.trackmate.tracking.sparselap import SparseLAPTrackerFactory
from fiji.plugin.trackmate.tracking import LAPUtils
from ij import IJ, ImagePlus, VirtualStack
#from ij.plugin import FolderOpener
from ij.process import ImageConverter

import fiji.plugin.trackmate.visualization.hyperstack.HyperStackDisplayer as HyperStackDisplayer
import fiji.plugin.trackmate.visualization.TrackMateModelView as TrackMateModelView
import fiji.plugin.trackmate.features.track.TrackIndexAnalyzer as TrackIndexAnalyzer
import fiji.plugin.trackmate.visualization.PerTrackFeatureColorGenerator as PerTrackFeatureColorGenerator
import fiji.plugin.trackmate.features.FeatureFilter as FeatureFilter
import fiji.plugin.trackmate.features.track.TrackDurationAnalyzer as TrackDurationAnalyzer
import fiji.plugin.trackmate.action.CaptureOverlayAction as CaptureOverlayAction

import sys, os, glob, re

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
datasetName = 'C6'
inputDir = '/home/nwalczak/workspace/elm/data/9_29_17/not_overlayed/C6/'
outputDir = '/home/nwalczak/workspace/elm/data/9_29_17/not_overlayed/test_output'
numChannels = 4;
numZ = 1;
noZInFile = True;

currChan = 3;
currZ = 0;

# Get currently selected image
#imp = WindowManager.getCurrentImage()
#imp = IJ.openImage('http://fiji.sc/samples/FakeTracks.tif')
#fo = FolderOpener()


imgFiles = glob.glob(os.path.join(inputDir, "*.tif"))
# Ensure we have tifs
if (len(imgFiles) < 1):
    print "No tif files found in input directory!  Input dir: " + inputDir
    exit

sort_nicely(imgFiles)
# Get info about image dimensions - needed for creating stacks
firstImage = IJ.openImage(imgFiles[0]);
imgWidth = firstImage.getWidth();
imgHeight = firstImage.getHeight();

# Count how many images we have for each channel/Z slice
imgFileCats = [[[] for z in range(numZ)] for c in range(numChannels)]
for c in range(0, numChannels):
    chanStr = 'ch%(channel)02d' % {"channel" : c};
    for z in range(0, numZ):
        zStr =  'z%(depth)02d' % {"depth" : z};
        count = 0;
        for imgPath in imgFiles:
            fileName = os.path.basename(imgPath)
            if chanStr in fileName and (noZInFile or zStr in fileName):
                imgFileCats[c][z].append(fileName)

# Load all images
images = [[0 for z in range(numZ)] for c in range(numChannels)]
#opener = Opener;
for c in range(0, numChannels):
    for z in range(0, numZ):
        imSeq = VirtualStack(imgWidth, imgHeight, firstImage.getProcessor().getColorModel(), inputDir)
        for fileName in imgFileCats[c][z]:
            imSeq.addSlice(fileName);
        images[c][z] = ImagePlus()
        images[c][z].setStack(imSeq)
        images[c][z].setTitle(datasetName + ", channel " + str(c) + ", z " + str(z))

#imp = ImagePlus()
#imp = fo.open(inputDir);
imp = images[currChan][currZ]

# Check for color images, and convert

if (imp.getType() != ImagePlus.GRAY8):
    toGray = ImageConverter(imp)
    toGray.convertToGray8()
    print imp.getType()

print "Channels: " + str(imp.getNChannels())
print "Slices: " + str(imp.getNSlices())
print "Frames: " + str(imp.getNFrames())

if (imp.getNSlices() > imp.getNFrames()):
    imp.setDimensions(imp.getNChannels(), imp.getNFrames(), imp.getNSlices())
    
imp.show()

#----------------------------
# Create the model object now
#----------------------------

# Some of the parameters we configure below need to have
# a reference to the model at creation. So we create an
# empty model now.

model = Model()

# Send all messages to ImageJ log window.
model.setLogger(Logger.IJ_LOGGER)

#------------------------
# Prepare settings object
#------------------------

settings = Settings()
settings.setFrom(imp)

# Configure detector - We use the Strings for the keys
settings.detectorFactory = LogDetectorFactory()
settings.detectorSettings = {
    'DO_SUBPIXEL_LOCALIZATION' : True,
    'RADIUS' : 0.01,
    'TARGET_CHANNEL' : 0,
    'THRESHOLD' : 0.5,
    'DO_MEDIAN_FILTERING' : True,
}

# Configure spot filters - Classical filter on quality
filter1 = FeatureFilter('QUALITY', 10, True)
settings.addSpotFilter(filter1)

# Configure tracker - We want to allow merges and fusions
settings.trackerFactory = SparseLAPTrackerFactory()
settings.trackerSettings = LAPUtils.getDefaultLAPSettingsMap() # almost good enough
settings.trackerSettings['ALLOW_TRACK_SPLITTING'] = False
settings.trackerSettings['ALLOW_TRACK_MERGING'] = False

# Configure track analyzers - Later on we want to filter out tracks
# based on their displacement, so we need to state that we want
# track displacement to be calculated. By default, out of the GUI,
# not features are calculated.

# The displacement feature is provided by the TrackDurationAnalyzer.

settings.addTrackAnalyzer(TrackDurationAnalyzer())

# Configure track filters - We want to get rid of the two immobile spots at
# the bottom right of the image. Track displacement must be above 10 pixels.

filter2 = FeatureFilter('TRACK_DISPLACEMENT', 1, True)
#settings.addTrackFilter(filter2)


#-------------------
# Instantiate plugin
#-------------------

trackmate = TrackMate(model, settings)

#--------
# Process
#--------

ok = trackmate.checkInput()
if not ok:
    sys.exit(str(trackmate.getErrorMessage()))

ok = trackmate.process()
if not ok:
    sys.exit(str(trackmate.getErrorMessage()))


#----------------
# Display results
#----------------

selectionModel = SelectionModel(model)
displayer =  HyperStackDisplayer(model, selectionModel, imp)
displayer.render()
displayer.refresh()
displayer.setDisplaySettings(TrackMateModelView.KEY_TRACK_COLORING, PerTrackFeatureColorGenerator(model, TrackIndexAnalyzer.TRACK_INDEX ))
displayer.setDisplaySettings(TrackMateModelView.KEY_TRACK_DISPLAY_MODE, TrackMateModelView.TRACK_DISPLAY_MODE_LOCAL_BACKWARD_QUICK)
displayer.setDisplaySettings(TrackMateModelView.KEY_TRACK_DISPLAY_DEPTH, 2)

# Echo results with the logger we set at start:
model.getLogger().log(str(model))

# The feature model, that stores edge and track features.
fm = model.getFeatureModel()

coa = CaptureOverlayAction()
coa.execute(trackmate)

IJ.saveAs('avi', os.path.join(outputDir, "out.avi"))

# Write output for tracks
numTracks = model.getTrackModel().trackIDs(True).size();
tCount = 0
for tId in model.getTrackModel().trackIDs(True):
    print "Writing track " + str(tId) + " (" + str(tCount) + " of " + str(numTracks) + ")..."
    tCount += 1
    # Ensure tracks dir exists
    trackOut = os.path.join(outputDir, "tracks")
    if not os.path.exists(trackOut):
        os.makedirs(trackOut)
    # Create output file
    trackOut = os.path.join(trackOut, "track_" + str(tId) + ".out")
    trackFile = open(trackOut, 'w')
    
    # Fetch the track feature from the feature model.
    v = fm.getTrackFeature(tId, 'TRACK_MEAN_SPEED')
    
    trackFile.writelines('Track ' + str(tId) + ': mean velocity = ' + str(v) + ' ' + model.getSpaceUnits() + '/' + model.getTimeUnits() + '\n')
    trackFile.writelines('Frame, T, X, Y, Z, Quality, SNR, Mean Intensity' + '\n')   
    track = model.getTrackModel().trackSpots(tId)
    
    # Write spot data
    for spot in track:
        # Fetch spot features directly from spot. 
        x=spot.getFeature('POSITION_X')
        y=spot.getFeature('POSITION_Y')
        z=spot.getFeature('POSITION_Z')
        f=spot.getFeature('FRAME')
        t=spot.getFeature('POSITION_T')
        q=spot.getFeature('QUALITY')
        snr=spot.getFeature('SNR') 
        mean=spot.getFeature('MEAN_INTENSITY')
        #print spot.echo()
        trackFile.write(str(f) + ', ' + str(t) + ', ' + str(x) + ', ' + str(y) + ', ' + str(z) + ', ' + str(q) + ',  ' + str(snr) + ', ' + str(mean) + '\n')
    
    trackFile.close()
        
        
        
        