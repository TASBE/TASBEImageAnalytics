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
elmSegPath = "elmSegPath"
scopeProperties = "scopeProperties"
numChannels = "numChannels"
numTimesteps = "numTimesteps"
numZ = "numZ"
numT = "numT"
noZInFile = "noZInFile"
noTInFile = "noTInFile"
chanLabel ="ChanLabel"
chansToSkip = "ChansToSkip"
inputDir = "inputDir"
outputDir = "outputDir"
analysisRoi = "analysisRoi"
dsNameIdx = "dsNameIdx"
pixelHeight = "pixelHeight"
pixelDepth = "pixelDepth"
pixelWidth = "pixelWidth"
wellNames = "wellNames"
debugOutput = "debugOutput"
pcloudColorThresh = "pcloudColorThresh"
pcloudExclusionX  = "pcloudExclusionX"
pcloudExclusionY  = "pcloudExclusionY"

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
        self.params[numChannels] = 4;
        self.params[numZ] = 1;
        self.params[numT] = 1;
        self.params[noZInFile] = True;
        self.params[noTInFile] = True;
        self.params[chanLabel] = [SKIP, YELLOW, BLUE, BRIGHTFIELD];
        self.params[chansToSkip] = [];
        self.params[inputDir] = '';
        self.params[outputDir] = '';
        # We need to avoid the scale bar in the bottom of the image, so set a roi that doesn't include it
        #self.params[analysisRoi] = [0,0,512,480]
        #self.params[analysisRoi] = [0,0,1024,980]
        # Determines what index the dataset name is within the tokenized filename
        self.params[dsNameIdx] = 4;
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
            if isinstance(self.params[key], list):
                print("\t" + key + ":\t" + ", ".join(self.params[key]))
            elif isinstance(self.params[key], int) or isinstance(self.params[key], float):
                print("\t" + key + ":\t" + str(self.params[key]))
            else:
                print("\t" + key + ":\t" + self.params[key])
        print("")

    ###
    #  Get the Z string in the filename, given the current Z and using the configs max num Z
    ###
    def getZStr(self, z):
        if self.params[numZ] < 100:
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
        if self.isCytation:
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
        
        cMatch = cStr in fileName
        zMatch = self.getValue(noZInFile) or zStr in fileName
        if self.isCytation:
            t = t + 1 # Cytation starts at a timestep of 1, so offset time indices
            tStr = self.getTStr(t) 
            tMatch = self.getValue(noTInFile) or tStr == os.path.splitext(fileName)[0].split("_")[-1]
        else:
            tMatch = self.getValue(noTInFile) or tStr in fileName
            
        return cMatch and zMatch and tMatch


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
            elif option == pcloudColorThresh.lower():
                self.params[pcloudColorThresh] = int(cfgParser.get(cfgSection, option))
            elif option == pcloudExclusionX.lower():
                self.params[pcloudExclusionX] = int(cfgParser.get(cfgSection, option))
            elif option == pcloudExclusionY.lower():
                self.params[pcloudExclusionY] = int(cfgParser.get(cfgSection, option))
            elif option == noZInFile.lower():
                self.params[noZInFile]  = cfgParser.get(cfgSection, option) == "True"
            elif option == noTInFile.lower():
                self.params[noTInFile]  = cfgParser.get(cfgSection, option) == "True"
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
            elif option == dsNameIdx.lower():
                self.params[dsNameIdx] = int(cfgParser.get(cfgSection, option))
            elif option == wellNames.lower():
                toks = cfgParser.get(cfgSection, option).split(",")
                self.params[wellNames] = []
                for t in toks:
                    self.params[wellNames].append(t)
            elif option == debugOutput.lower():
                self.params[debugOutput] = cfgParser.get(cfgSection, option) == "True"
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

        cytationMetadata = TiffTags.getTag(pathToImage, CYTATION_METADATA_TIFF_TAG, 1)
        cytationMetadata = cytationMetadata[0:cytationMetadata.rfind('>') + 1]
        # Check that we have metdata
        if not cytationMetadata:
            return
        xmlRoot = ElementTree.fromstring(cytationMetadata)
        imgAcq = xmlRoot.find("ImageAcquisition")
        imgRef = xmlRoot.find("ImageReference")

        # if we don't have the right XML elements, maybe not Cytation
        if imgAcq is None or imgRef is None:
            return
        
        self.isCytation = True
        
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
            if ch in self.params[chansToSkip]:
                self.params[chanLabel].append(SKIP)
            else:
                self.params[chanLabel].append(ch)


    ####
    #
    # Replace any channel labels with skip, if necessary
    #
    ####    
    def checkSkipChans(self):
        for skipChan in self.params[chansToSkip]:
            for i in range(0, len(self.params[chanLabel])):
                if skipChan == self.params[chanLabel][i]:
                    self.params[chanLabel][i] = SKIP


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
            newChan = chan.get("LUTName")
            if newChan in self.params[chansToSkip]:
                chanNames.append(SKIP)
            else:
                chanNames.append(chan.get("LUTName"))
        self.params[chanLabel] = chanNames

        # Pull dimension info from XML 
        dimsEle = imgDescEle.find("Dimensions")
        for dimEle in dimsEle.getchildren():
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


