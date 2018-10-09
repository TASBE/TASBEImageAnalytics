from ij import IJ, ImagePlus, ImageStack
from ij.process import ImageConverter, AutoThresholder

from fiji.plugin.trackmate import Model, Settings, TrackMate, SelectionModel, Logger
from fiji.plugin.trackmate.detection import ThresholdDetectorFactory, LocalThresholdDetectorFactory
from fiji.plugin.trackmate.tracking.sparselap import SparseLAPTrackerFactory
from fiji.plugin.trackmate.tracking import LAPUtils

from fiji.plugin.trackmate.visualization import TrackMateModelView, PerTrackFeatureColorGenerator, SpotColorGeneratorPerTrackFeature

import fiji.plugin.trackmate.visualization.hyperstack.HyperStackDisplayer as HyperStackDisplayer
import fiji.plugin.trackmate.features.FeatureFilter as FeatureFilter

#import fiji.plugin.trackmate.features.track.TrackDurationAnalyzer as TrackDurationAnalyzer
from fiji.plugin.trackmate.features.track import TrackBranchingAnalyzer, TrackIndexAnalyzer, TrackLocationAnalyzer, TrackSpeedStatisticsAnalyzer, TrackDurationAnalyzer
from fiji.plugin.trackmate.features.spot import SpotIntensityAnalyzerFactory, SpotContrastAndSNRAnalyzerFactory

import fiji.plugin.trackmate.action.CaptureOverlayAction as CaptureOverlayAction

from java.util import HashMap


import fiji.plugin.trackmate.tracking.TrackerKeys as TrackerKeys

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
        print "No MetaData directory in input dir! Can't read Leica properties!"
        metadataExists = False;
    elif cfg.isCytation or (cfg.getValue(ELMConfig.imgType) == "png"):
        metadataExists = False;

    # Process each well
    trackDat = {}
    for wellName in uniqueNames:
        # Check to see if we should ignore this well
        if cfg.getValue(ELMConfig.wellNames):
            if not wellName in cfg.getValue(ELMConfig.wellNames):
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
        trackDat[wellName] = processDataset(cfg, wellName, dsImgFiles)
        end = time.time()
        print("Processed well " + wellName + " in " + str(end - start) + " s")
        print("\n\n")

    # Create output file
    trackOut = os.path.join(cfg.getValue(ELMConfig.outputDir), "AlltrackSummary.csv")
    trackFile = open(trackOut, 'w')
    date = os.path.basename(cfg.getValue(ELMConfig.outputDir))
    # Fetch the track feature from the feature model.
    trackFile.write('Date, Wellname, Num Tracks, Mean Duration, Max Duration, Min Duration, Mean Avg Total Intensity, Max Avg Total Intensity, Min Avg Total Intensity \n')
    for wellName in trackDat:
        tracks = trackDat[wellName]
        numTracks = len(tracks);
        durations = [sys.maxint,0,0]
        avgTotalItensities = [sys.maxint,0,0]
        for tId in tracks:
            dur = float(tracks[tId][1])
            avgTotalInt = float(tracks[tId][2])
            if (dur < durations[0]):
                durations[0] = dur
            if (dur > durations[1]):
                durations[1] = dur
            durations[2] += dur
            
            if (avgTotalInt < avgTotalItensities[0]):
                avgTotalItensities[0] = avgTotalInt
            if (avgTotalInt > avgTotalItensities[1]):
                avgTotalItensities[1] = avgTotalInt
            avgTotalItensities[2] += avgTotalInt    
            
        durations[2] /= numTracks
        avgTotalItensities[2] /= numTracks    
        dat = [date, wellName, str(numTracks), str(durations[2]), str(durations[1]), str(durations[0]), str(avgTotalItensities[2]), str(avgTotalItensities[1]), str(avgTotalItensities[0])]
        trackFile.write(','.join(dat) + '\n')
    trackFile.close()

####
#
#
####
def processDataset(cfg, datasetName, imgFiles):
    datasetPath = os.path.join(cfg.getValue(ELMConfig.outputDir), datasetName)

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

    # Process images
    trackDat = processImages(cfg, datasetName, datasetPath, imgFileCats)
    return trackDat

####
#
#  All of the processing that happens for each image
#
####
def processImages(cfg, wellName, wellPath, images):
    firstImage = IJ.openImage(images[0][0][0][0]);
    imgWidth = firstImage.getWidth();
    imgHeight = firstImage.getHeight();
    
    for c in range(0, cfg.getValue(ELMConfig.numChannels)):
        if cfg.getValue(ELMConfig.chanLabel)[c] in cfg.getValue(ELMConfig.chansToSkip):
            continue;

        imSeq = ImageStack(imgWidth, imgHeight)
        totalHist = []
        for z in range(0, cfg.getValue(ELMConfig.numZ)):
            for t in range(0, cfg.getValue(ELMConfig.numT)):
                currIP = IJ.openImage(images[c][z][t][0])
                imgType = currIP.getType()
                if not imgType == ImagePlus.GRAY8: 
                    toGray = ImageConverter(currIP)
                    toGray.convertToGray8()
                imSeq.addSlice(currIP.getProcessor());
                imgStats = currIP.getStatistics()
                currHist = imgStats.getHistogram()
                if not totalHist:
                    for i in range(len(currHist)):
                        totalHist.append(currHist[i])
                else:
                    for i in range(len(currHist)):
                        totalHist[i] += currHist[i]

        
        if cfg.hasValue(ELMConfig.thresholdFromWholeRange) and cfg.getValue(ELMConfig.thresholdFromWholeRange) == True:
            threshMethod = "Otsu" # Default works very poorly for this data
            if cfg.hasValue(ELMConfig.thresholdMethod):
                threshMethod = cfg.getValue(ELMConfig.thresholdMethod)
            thresholder = AutoThresholder()
            computedThresh = thresholder.getThreshold(threshMethod, totalHist)
            cfg.setValue(ELMConfig.imageThreshold, computedThresh)
            print("\tComputed threshold from total hist (" + threshMethod + "): " + str(computedThresh))
            print()
        else:
            print("\tUsing threshold computed on individual images!")
            print()
            computedThresh = -1
        
        chanName = cfg.getValue(ELMConfig.chanLabel)[c]
        
        imp = ImagePlus()
        imp.setStack(imSeq)
        imp.setDimensions(1, 1, cfg.getValue(ELMConfig.numT))
        imp.setTitle(wellName + ", channel " + str(c))

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
        
        dbgPath = os.path.join(wellPath, 'debugImages')
        if not os.path.exists(dbgPath):
            os.makedirs(dbgPath)
        
        # Configure detector - We use the Strings for the keys
        settings.detectorFactory = ThresholdDetectorFactory()
        settings.detectorSettings = {
            'THRESHOLD' : computedThresh,
            'DEBUG_MODE' : True,
            'DEBUG_OUTPATH' : dbgPath
        }
        
        settings.detectorFactory = LocalThresholdDetectorFactory()
        settings.detectorSettings = {
            'THRESHOLD' : computedThresh,
            'DEBUG_MODE' : True,
            'DEBUG_OUTPATH' : dbgPath
        }
        
        # Configure spot filters - Classical filter on quality
        filter1 = FeatureFilter('QUALITY', 50, True)
        settings.addSpotFilter(filter1)
        
        # Configure tracker - We want to allow merges and fusions
        settings.trackerFactory = SparseLAPTrackerFactory()
        settings.trackerSettings = LAPUtils.getDefaultLAPSettingsMap() # almost good enough
        
        # Linking
        settings.trackerSettings[TrackerKeys.KEY_LINKING_MAX_DISTANCE] = 120.0; # in pixels
        
        linkFeaturePenalties = HashMap();
        linkFeaturePenalties['Area'] = 1.0
        linkFeaturePenalties['Circ.'] = 1.0
        linkFeaturePenalties['Mean'] = 1.0
         
        settings.trackerSettings[TrackerKeys.KEY_LINKING_FEATURE_PENALTIES] = linkFeaturePenalties;
        # Gap closing
        settings.trackerSettings[TrackerKeys.KEY_ALLOW_GAP_CLOSING] =  True;
        settings.trackerSettings[TrackerKeys.KEY_GAP_CLOSING_MAX_FRAME_GAP] =  8;
        settings.trackerSettings[TrackerKeys.KEY_GAP_CLOSING_MAX_DISTANCE] =  120.0; # in pixels
        #settings.trackerSettings[TrackerKeys.KEY_GAP_CLOSING_FEATURE_PENALTIES] =  new HashMap<>(DEFAULT_GAP_CLOSING_FEATURE_PENALTIES));
        # Track splitting
        settings.trackerSettings[TrackerKeys.KEY_ALLOW_TRACK_SPLITTING] =  False;
        settings.trackerSettings[TrackerKeys.KEY_SPLITTING_MAX_DISTANCE] =  45.0; # in pixels
        #settings.trackerSettings[TrackerKeys.KEY_SPLITTING_FEATURE_PENALTIES] =  new HashMap<>(DEFAULT_SPLITTING_FEATURE_PENALTIES));
        # Track merging
        settings.trackerSettings[TrackerKeys.KEY_ALLOW_TRACK_MERGING] =  False;
        settings.trackerSettings[TrackerKeys.KEY_MERGING_MAX_DISTANCE] =  45.0; # in pixels
        #settings.trackerSettings[TrackerKeys.KEY_MERGING_FEATURE_PENALTIES] =  new HashMap<>(DEFAULT_MERGING_FEATURE_PENALTIES));
        # Others
        settings.trackerSettings[TrackerKeys.KEY_BLOCKING_VALUE] =  float("inf");
        settings.trackerSettings[TrackerKeys.KEY_ALTERNATIVE_LINKING_COST_FACTOR] =  1.05;
        settings.trackerSettings[TrackerKeys.KEY_CUTOFF_PERCENTILE] =  0.9;

        
        # Configure track analyzers - Later on we want to filter out tracks
        # based on their displacement, so we need to state that we want
        # track displacement to be calculated. By default, out of the GUI,
        # no features are calculated.
        
        # The displacement feature is provided by the TrackDurationAnalyzer.
        settings.addTrackAnalyzer(TrackDurationAnalyzer())
        settings.addTrackAnalyzer(TrackBranchingAnalyzer())
        settings.addTrackAnalyzer(TrackIndexAnalyzer())
        settings.addTrackAnalyzer(TrackLocationAnalyzer())
        settings.addTrackAnalyzer(TrackSpeedStatisticsAnalyzer())
        
        settings.addSpotAnalyzerFactory(SpotIntensityAnalyzerFactory())
        settings.addSpotAnalyzerFactory(SpotContrastAndSNRAnalyzerFactory())        
        
        # Configure track filters - We want to get rid of the two immobile spots at
        # the bottom right of the image. Track displacement must be above 10 pixels.
        #filter2 = FeatureFilter('TRACK_DISPLACEMENT', 1, True)
        #settings.addTrackFilter(filter2)
        
        
        print("Spot feature analyzers: " + settings.toStringFeatureAnalyzersInfo())
        
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
        
        print ("Processing " + chanName + "...")
        ok = trackmate.process()
        if not ok:
            sys.exit(str(trackmate.getErrorMessage()))
        
        
        #----------------
        # Display results
        #----------------
        print("Rendering...")
        
        # Set spot names based on track IDs
        # This allows track IDs to be displayed in the rendered video
        for tId in model.getTrackModel().trackIDs(True):
            trackSpots = model.getTrackModel().trackSpots(tId)
            for spot in trackSpots:
                spot.setName(str(tId))
        
        selectionModel = SelectionModel(model)
        displayer =  HyperStackDisplayer(model, selectionModel, imp)
        displayer.setDisplaySettings(TrackMateModelView.KEY_TRACK_COLORING, PerTrackFeatureColorGenerator(model, TrackIndexAnalyzer.TRACK_INDEX ))
        displayer.setDisplaySettings(TrackMateModelView.KEY_SPOT_COLORING, SpotColorGeneratorPerTrackFeature(model, TrackIndexAnalyzer.TRACK_INDEX ))
        displayer.setDisplaySettings(TrackMateModelView.KEY_DISPLAY_SPOT_NAMES, True)        
        displayer.setDisplaySettings(TrackMateModelView.KEY_TRACK_DISPLAY_MODE, TrackMateModelView.TRACK_DISPLAY_MODE_LOCAL_BACKWARD_QUICK)
        displayer.setDisplaySettings(TrackMateModelView.KEY_TRACK_DISPLAY_DEPTH, 2)
        displayer.render()
        displayer.refresh()
        
        # Echo results with the logger we set at start:
        model.getLogger().log(str(model))
        
        # The feature model, that stores edge and track features.
        fm = model.getFeatureModel()
        
        coa = CaptureOverlayAction(None)
        coa.execute(trackmate)
        
        IJ.saveAs('avi', os.path.join(wellPath, chanName + "_out.avi"))

        imp.close()
        displayer.clear()
        displayer.getImp().hide()
        displayer.getImp().close()
        coa.getCapture().hide()
        coa.getCapture().close()

        # Write output for tracks
        numTracks = model.getTrackModel().trackIDs(True).size();
        print "Writing track data for " + str(numTracks) + " tracks."
        trackDat = {} 
        for tId in model.getTrackModel().trackIDs(True):
            track = model.getTrackModel().trackSpots(tId)
            
            # Ensure track spots dir exists
            trackOut = os.path.join(wellPath, chanName + "_track_spots")
            if not os.path.exists(trackOut):
                os.makedirs(trackOut)
            # Create output file
            trackOut = os.path.join(trackOut, "track_" + str(tId) + ".csv")
            trackFile = open(trackOut, 'w')

            # Write Header
            header = 'Frame, '
            for feature in track.toArray()[0].getFeatures().keySet():
                if feature == 'Frame':
                    continue;
                header += feature + ", "
            header = header[0:len(header) - 2]
            header += '\n'
            trackFile.write(header)
            # Write spot data
            avgTotalIntensity = 0
            for spot in track:
                #print spot.echo()
                data = [str(spot.getFeature('FRAME'))]
                for feature in spot.getFeatures():
                    if feature == 'Frame':
                        continue;
                    elif feature == 'TOTAL_INTENSITY':
                        avgTotalIntensity += spot.getFeature(feature)
                    data.append(str(spot.getFeature(feature)))
                trackFile.write(','.join(data) + '\n')
            trackFile.close()
            avgTotalIntensity /= len(track)

            # Write out track stats
            # Make sure dir exists            
            trackOut = os.path.join(wellPath, chanName + "_tracks")
            if not os.path.exists(trackOut):
                os.makedirs(trackOut)
            # Create output file
            trackOut = os.path.join(trackOut, "track_" + str(tId) + ".csv")
            trackFile = open(trackOut, 'w')
            # Fetch the track feature from the feature model.
            header = ''
            for featName in fm.getTrackFeatureNames():
                header += featName + ", "
            header = header[0:len(header) - 2]
            header += '\n'
            trackFile.write(header)
            
            features = ''
            for featName in fm.getTrackFeatureNames():
                features += str(fm.getTrackFeature(tId, featName)) + ', '
            features = features[0:len(features) - 2]
            features += '\n'
            trackFile.write(features)
            trackFile.write('\n')
            trackFile.close()
            
            trackDat[tId] = [str(tId), str(fm.getTrackFeature(tId, 'TRACK_DURATION')), str(avgTotalIntensity), str(fm.getTrackFeature(tId, 'TRACK_START')), str(fm.getTrackFeature(tId, 'TRACK_STOP'))]

        # Create output file
        trackOut = os.path.join(wellPath, chanName + "_trackSummary.csv")
        trackFile = open(trackOut, 'w')
        # Fetch the track feature from the feature model.
        trackFile.write('Track Id, Duration, Avg Total Intensity, Start Frame, Stop Frame \n')
        for track in trackDat:
            trackFile.write(','.join(trackDat[track]) + '\n')
        trackFile.close()
    return trackDat

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
    imageJ = True
except NameError:
    imageJ = False
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
if imageJ:
    IJ.run("Quit")
else:
    exit(0)

