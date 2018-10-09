#Recognized colors
RED = "Red"
GREEN = "Green"
BLUE = "Blue"
YELLOW = "Yellow"
BRIGHTFIELD = "Gray"
SKIP = "Skip"

UM_AREA = "Area (um^2)"

import re, os,  ConfigParser
import xml.etree.ElementTree as ElementTree
import TIFF_Tags as TiffTags

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

# SECTION
cfgSection = "ImageJConfig"

# PARAMS
wellIdx = "wellIdx"
cIdx = "cIdx"
zIdx = "zIdx"
tIdx = "tIdx"
zList = "zList"
tList = "tList"
imgType = "imgType"
elmSegPath = "elmSegPath"
scopeProperties = "scopeProperties"
numChannels = "numChannels"
numTimesteps = "numTimesteps"
numZ = "numZ"
numT = "numT"
minT = "minT"
noZInFile = "noZInFile"
noTInFile = "noTInFile"
chanLabel ="ChanLabel"
chansToSkip = "ChansToSkip"
inputDir = "inputDir"
outputDir = "outputDir"
analysisRoi = "analysisRoi"
pixelHeight = "pixelHeight"
pixelDepth = "pixelDepth"
pixelWidth = "pixelWidth"
wellNames = "wellNames"
excludeWellNames = "excludeWellNames"
debugOutput = "debugOutput"
pcloudColorThresh = "pcloudColorThresh"
pcloudExclusionX  = "pcloudExclusionX"
pcloudExclusionY  = "pcloudExclusionY"
lowerRightExclusionX  = "lowerRightExclusionX"
lowerRightExclusionY  = "lowerRightExclusionY"
upperLeftExclusionX  = "upperLeftExclusionX"
upperLeftExclusionY  = "upperLeftExclusionY"
maxThreshRange = "maxThreshRange"
thresholdMethod = "thresholdMethod"
defaultThreshold = "defaultThreshold" # if set, use this value to threshold if the maxThreshRange is hit
imageThreshold = "imageThreshold"
areaMaxPercentThreshold = "areaMaxPercentThreshold" # Remove blobs with area < areaMaxPercentThreshold * maxArea
areaAbsoluteThreshold = "areaAbsoluteThreshold" # Remove blobs with area < areaAbsoluteThreshold
lutPath = "lutPath"
createSegMask = "createSegMask"
invertLut = "invertLut"
thresholdFromWholeRange = "thresholdFromWholeRange"

CYTATION_METADATA_TIFF_TAG = 270

####
#
#  The Config Class - storing configuration info
#
####
class ConfigParams:
    params = dict()
    
    isCytation = False

    ###
    #
    ### 
    def __init__(self):
        self.params[pixelHeight] = 1
        self.params[pixelWidth] = 1
        self.params[imgType] = "tif"
        self.params[numChannels] = 4;
        self.params[numZ] = 1;
        self.params[numT] = 1;
        self.params[minT] = 0;
        self.params[noZInFile] = True;
        self.params[noTInFile] = True;
        self.params[chanLabel] = [SKIP, YELLOW, BLUE, BRIGHTFIELD];
        self.params[chansToSkip] = [];
        self.params[inputDir] = '';
        self.params[outputDir] = '';
        self.params[maxThreshRange] = 230
        self.params[lutPath] = '/home/nwalczak/workspace/fiji/Fiji.app/luts/glasbey_on_dark.lut'
        # Determines what index the dataset name is within the tokenized filename
        #self.params[pixelHeight] = 1; # in micrometers
        #self.params[pixelWidth] = 1; # in micrometers
        #self.params[wellNames] = [] # List of well names to process, empty implies process all
        self.params[debugOutput] =  False; # If true, additional info will be output



    ###
    #
    ###
    def isCytationConfig(self):
        return self.isCytation



    ###
    #
    ###
    def getValue(self, key):
        if key in self.params:
            return self.params[key]
        else:
            return []


    ###
    #
    ###
    def setValue(self, key, value):
        self.params[key] = value

    ###
    #
    ###
    def hasValue(self, key):
        return key in self.params

    ###
    #
    ###
    def printCfg(self):
        print("Using Config:")
        paramKeys = sorted(self.params.keys())
        for key in paramKeys:
            if key == zList or key == tList: # skip large config params
                continue;
            if isinstance(self.params[key], list):
                stingList = []
                for item in self.params[key]:
                    stingList.append(str(item))
                print("\t" + key + ":\t" + ", ".join(stingList))
            elif isinstance(self.params[key], int) or isinstance(self.params[key], float):
                print("\t" + key + ":\t" + str(self.params[key]))
            else:
                print("\t" + key + ":\t" + self.params[key])
        print("")

    ###
    #  Get the Z string in the filename, given the current Z and using the configs max num Z
    ###
    def getZStr(self, z):
        if self.params[imgType] == "png":
            return '%(depth)0.1f' % {"depth" : self.params[zList][z]}
        elif self.params[numZ] < 100:
            return 'z%(depth)02d' % {"depth" : z};
        else:
            return 'z%(depth)03d' % {"depth" : z};
        
    ###
    #  Get the channel string in the filename, given the current C and using the configs channel labels
    ###
    def getCStr(self, c):
        if self.isCytation:
            return self.getValue(chanLabel)[c];
        else:
            return 'ch%(channel)02d' % {"channel" : c};


    ###
    #  Get the T string in the filename, given the current T and using the configs max num T
    ###
    def getTStr(self, t):
        if self.params[imgType] == "png":
            return '%(time)0.1f' % {"time" : self.params[tList][t]}
        elif self.isCytation:
            if self.params[numT] < 100:
                return '%(time)02d' % {"time" : t};
            else:
                return '%(time)03d' % {"time" : t};
        else:
            if self.params[numT] < 100:
                return 't%(time)02d' % {"time" : t};
            else:
                return 't%(time)03d' % {"time" : t};


    ###
    #  Determine if the filename is a match for the given channel, z and time
    #  indexes.
    ###
    def matchFilename(self, fileName, c, z, t):
        cStr = self.getCStr(c)
        zStr = self.getZStr(z)
        tStr = self.getTStr(t)
        
        fileToks = os.path.splitext(fileName)[0].split("_")
        
        if self.params[imgType] == "png":
            cMatch = True
            zMatch = zStr == fileToks[self.getValue(zIdx)]
            tMatch = tStr == fileToks[self.getValue(tIdx)] 
            return cMatch and zMatch and tMatch
        
        cMatch = cStr in fileName
        zMatch = self.getValue(noZInFile) or zStr in fileName
        if self.isCytation:
            t = t + 1 # Cytation starts at a timestep of 1, so offset time indices
            tStr = self.getTStr(t) 
            tMatch = self.getValue(noTInFile) or tStr == fileToks[-1]
        else:
            tMatch = self.getValue(noTInFile) or tStr in fileName
            
        return cMatch and zMatch and tMatch


    ###
    #  Determine the z,t values from the filename for PNG images
    ###
    def getZTFromFilename(self, fileName):
        if not self.params[imgType] == "png":
            print "Error: calling getZTFromFilename on non-PNG image!"
            return
        
        fileToks = os.path.splitext(fileName)[0].split("_")
        if self.getValue(noZInFile):
            zVal = 0
        else:
            zVal = float(fileToks[self.getValue(zIdx)])
        tVal = float(fileToks[self.getValue(tIdx)])
        return self.params[zList].index(zVal), self.params[tList].index(tVal)
    
    ###
    #  Determine the c,z,t values from the filename for PNG images
    ###
    def getCZTFromFilename(self, fileName):
        if not self.hasValue(cIdx):
            print "Error: calling getCZTFromFilename with no defined cIdx!"
            quit(-1)
        if not self.params[noZInFile] and not self.hasValue(zIdx):
            print "Error: calling getCZTFromFilename with no defined zIdx!"
            quit(-1)
        if not self.params[noTInFile] and not self.hasValue(tIdx):
            print "Error: calling getCZTFromFilename with no defined tIdx!"
            quit(-1)

        fileToks = os.path.splitext(fileName)[0].split("_")
        cStr = fileToks[self.params[cIdx]]
        try:
            chan = int(cStr.replace('ch',''))
        except:
            chan = self.getValue(chanLabel).index(cStr)

        if zIdx not in self.params:
            z = 0
        else:
            zStr = fileToks[self.params[zIdx]]
            z = int(zStr.replace('z',''))

        if tIdx not in self.params:
            t = 0;
        else:
            tStr = fileToks[self.params[tIdx]]
            t = int(tStr.replace('t',''))
            if self.hasValue(minT):
                t -= self.getValue(minT)
        return chan, z, t



    ###
    #
    ###
    def loadConfig(self, cfgPath):
        cfgParser = ConfigParser.RawConfigParser(allow_no_value=True)
        cfgParser.readfp(open(cfgPath))

        if not cfgParser.has_section(cfgSection):
            print "Config file doesn't contain [" + cfgSection + "] section! Path: " + cfgPath;
            return False

        if cfgParser.has_option(cfgSection, numChannels.lower()) :
            numChan = int(cfgParser.get(cfgSection, numChannels.lower()))
        else:
            numChan = self.params[numChannels]

        for optionRaw in cfgParser.options(cfgSection):
            option = optionRaw.lower()
            if option == inputDir.lower():
                self.params[inputDir] = cfgParser.get(cfgSection, option)
            elif option == elmSegPath.lower():
                self.params[elmSegPath] = cfgParser.get(cfgSection, option)
            elif option == wellIdx.lower():
                val = cfgParser.get(cfgSection,option)
                if ',' in val:
                    toks = cfgParser.get(cfgSection, option).split(",")
                    self.params[wellIdx] = []
                    for tok in toks:
                        self.params[wellIdx].append(int(tok))
                else:
                    self.params[wellIdx] = int(val)
            elif option == imgType.lower():
                inputImgType = cfgParser.get(cfgSection, option)
                if not inputImgType == "tif" and not inputImgType == "png":
                    print "Error: unsupported image type! Expected png or tif, received: " + inputImgType
                self.params[imgType] = inputImgType
            elif option == outputDir.lower():
                self.params[outputDir] = cfgParser.get(cfgSection, option)
            elif option == numChannels.lower():
                self.params[numChannels] = int(cfgParser.get(cfgSection, option))
            elif option == numTimesteps.lower():
                self.params[numTimesteps] = int(cfgParser.get(cfgSection, option))
            elif option == numZ.lower():
                self.params[numZ] = int(cfgParser.get(cfgSection, option))
            elif option == numT.lower():
                self.params[numT] = int(cfgParser.get(cfgSection, option))
            elif option == minT.lower():
                self.params[minT] = int(cfgParser.get(cfgSection, option))
            elif option == tIdx.lower():
                self.params[tIdx] = int(cfgParser.get(cfgSection, option))
            elif option == zIdx.lower():
                self.params[zIdx] = int(cfgParser.get(cfgSection, option))
            elif option == pcloudColorThresh.lower():
                self.params[pcloudColorThresh] = int(cfgParser.get(cfgSection, option))
            elif option == pcloudExclusionX.lower() or option == lowerRightExclusionX.lower():
                self.params[lowerRightExclusionX] = int(cfgParser.get(cfgSection, option))
            elif option == pcloudExclusionY.lower() or option == lowerRightExclusionY.lower():
                self.params[lowerRightExclusionY] = int(cfgParser.get(cfgSection, option))
            elif option == upperLeftExclusionX.lower():
                self.params[upperLeftExclusionX] = int(cfgParser.get(cfgSection, option))
            elif option == upperLeftExclusionY.lower():
                self.params[upperLeftExclusionY] = int(cfgParser.get(cfgSection, option))
            elif option == maxThreshRange.lower():
                self.params[maxThreshRange] = int(cfgParser.get(cfgSection, option))
            elif option == thresholdMethod.lower():
                self.params[thresholdMethod] = cfgParser.get(cfgSection, option)
            elif option == defaultThreshold.lower():
                self.params[defaultThreshold] = int(cfgParser.get(cfgSection, option))
            elif option == imageThreshold.lower():
                self.params[imageThreshold] = int(cfgParser.get(cfgSection, option))
            elif option == areaMaxPercentThreshold.lower():
                self.params[areaMaxPercentThreshold] = float(cfgParser.get(cfgSection, option))
            elif option == areaAbsoluteThreshold.lower():
                self.params[areaAbsoluteThreshold] = float(cfgParser.get(cfgSection, option))
            elif option == noZInFile.lower():
                self.params[noZInFile]  = cfgParser.get(cfgSection, option) == "True"
            elif option == noTInFile.lower():
                self.params[noTInFile]  = cfgParser.get(cfgSection, option) == "True"
            elif option == lutPath.lower():
                self.params[lutPath]  = cfgParser.get(cfgSection, option)
            elif option == chansToSkip.lower():
                toks = cfgParser.get(cfgSection, option).split(",")
                self.params[chansToSkip] = []
                for tok in toks:
                    self.params[chansToSkip].append(tok.strip())
            elif option == chanLabel.lower():
                toks = cfgParser.get(cfgSection, option).split(",")
                if not len(toks) == numChan:
                    print "Improper value for chanLabel config, expected " + str(numChan) + " comma separated values!  Received " + str(len(toks))
                self.params[chanLabel] = []
                for tok in toks:
                    self.params[chanLabel].append(tok.strip())
            elif option == analysisRoi.lower():
                toks = cfgParser.get(cfgSection, option).split(",")
                if not len(toks) == 4:
                    print "Improper value for analysisRoi config, expected " + 4+ " comma separated values!  Received " + str(len(toks))
                self.params[analysisRoi] = [toks[0],toks[1],toks[2],toks[3]]
            elif option == wellNames.lower():
                toks = cfgParser.get(cfgSection, option).split(",")
                self.params[wellNames] = []
                for t in toks:
                    self.params[wellNames].append(t)
            elif option == excludeWellNames.lower():
                toks = cfgParser.get(cfgSection, option).split(",")
                self.params[excludeWellNames] = []
                for t in toks:
                    self.params[excludeWellNames].append(t)
            elif option == debugOutput.lower():
                self.params[debugOutput] = cfgParser.get(cfgSection, option) == "True"
            elif option == createSegMask.lower():
                self.params[createSegMask] = cfgParser.get(cfgSection, option) == "True"
            elif option == invertLut.lower():
                self.params[invertLut] = cfgParser.get(cfgSection, option) == "True"
            elif option == thresholdFromWholeRange.lower():
                self.params[thresholdFromWholeRange] = cfgParser.get(cfgSection, option) == "True"
            else:
                print "Warning, unrecognized config option: " + option   
        
        return True


    ####
    #
    # Given a path to a an image, check to see if it contains the Cytation
    # metadata in a TIF tag. 
    #
    ####    
    def checkCytationMetadata(self, pathToImage):
        global RED, GREEN, BLUE, YELLOW 
        global BRIGHTFIELD

        if self.params[imgType] == "png":
            return
        
        print "Checking Cytation!"
        
        cytationMetadata = TiffTags.getTag(pathToImage, CYTATION_METADATA_TIFF_TAG, 1)
        cytationMetadata = cytationMetadata[0:cytationMetadata.rfind('>') + 1]
        # Check that we have metdata
        if not cytationMetadata:
            print "Found no metadata!"
            return
        xmlRoot = ElementTree.fromstring(cytationMetadata)
        imgAcq = xmlRoot.find("ImageAcquisition")
        imgRef = xmlRoot.find("ImageReference")

        # if we don't have the right XML elements, maybe not Cytation
        if imgAcq is None or imgRef is None:
            print "Found wrong metadata!"
            return
        
        self.isCytation = True
        
        print "Found Cytation!"
        
        # Need to alter channel names, as the Cytation uses different names
        GREEN = "GFP"
        BLUE = "BFP"
        YELLOW = "YFP"
        RED = "Texas Red"
        BRIGHTFIELD = "Phase Contrast"
        
        numXPix = int(imgAcq.find("PixelWidth").text)
        numYPix = int(imgAcq.find("PixelHeight").text)
        imgWidth = float(imgAcq.find("ImageWidthMicrons").text)
        imgHeight = float(imgAcq.find("ImageHeightMicrons").text)
            
        self.params[pixelWidth] = imgWidth / numXPix
        self.params[pixelHeight] = imgHeight / numYPix
        
        self.params[numZ] = int(imgRef.find("ZStackTotal").text)
        self.params[pixelDepth] = int(imgRef.find("ZStackStepSizeMicrons").text)
        self.params[numChannels] = int(imgRef.find("MeasurementTotal").text)



    ####
    #
    # Given a path of images, parse all Cytation metadata to get channel names
    #
    ####    
    def getCytationChanNames(self, imgFiles):
        chanNames = set()
        for imgF in imgFiles:
            cytationMetadata = TiffTags.getTag(imgF, CYTATION_METADATA_TIFF_TAG, 1)
            cytationMetadata = cytationMetadata[0:cytationMetadata.rfind('>') + 1]
            xmlRoot = ElementTree.fromstring(cytationMetadata)
            imgAcq = xmlRoot.find("ImageAcquisition")
            chanEle = imgAcq.find("Channel")
            chanNames.add(chanEle.get("Color"))
        if not len(chanNames) == self.getValue(numChannels):
            print "CytationChanNames: number of channels doesn't equal found names!"
            print "Num Expected Channels: " + str(self.getValue(numChannels))
            print "Found Names: " + chanNames
         
         
        self.params[chanLabel] = []
        for ch in chanNames:
            self.params[chanLabel].append(ch)


    ####
    #
    # Given a path to a Leica properties XML, read in some configuration
    #
    ####    
    def updateCfgWithXML(self, xmlFile):
        self.params[scopeProperties] = xmlFile
        xmlRoot = ElementTree.parse(xmlFile).getroot()
        imgEle = xmlRoot.find("Image")
        imgDescEle = imgEle.find("ImageDescription")

        # Pull channel info from XML
        chanEle = imgDescEle.find("Channels")
        self.params[numChannels] = len(chanEle.getchildren())
        chanNames = []
        for chan in chanEle.getchildren():
            chanNames.append(chan.get("LUTName"))
        self.params[chanLabel] = chanNames

        # Pull dimension info from XML 
        dimsEle = imgDescEle.find("Dimensions")
        for dimEle in dimsEle.getchildren():
            dimId = dimEle.get("DimID")
            # Ignore the time dimension; we don't need to do anything with it
            if dimId == 'T':
                continue;
            numElementsInDim = int(dimEle.get("NumberOfElements"))
            dimLength = float(dimEle.get("Length"))
            dimUnit = dimEle.get("Unit")
            lenMultiplier = self.getMultiplier(dimUnit) # Ensures length is in micrometers
            if dimEle.get("DimID") == "X":
                self.params[pixelWidth] = (dimLength * lenMultiplier) / numElementsInDim
            elif dimEle.get("DimID") == "Y":
                self.params[pixelHeight] = (dimLength * lenMultiplier) / numElementsInDim
            elif dimEle.get("DimID") == "Z":
                self.params[numZ] = numElementsInDim
                self.params[pixelDepth] = (dimLength * lenMultiplier) / numElementsInDim


    ####
    #
    # Get the multiplier to turn the given unit into micrometers
    #
    ####
    def getMultiplier(self, unit):
        mult = 1;
        if unit == "m":
            mult = 1e6;
        elif unit == "cm":
            mult = 1e4;
        elif unit == "mm":
            mult = 1e3;
        elif unit == "um":
            mult = 1;
        elif unit == "nm":
            mult = 1e-3;
        return mult;


