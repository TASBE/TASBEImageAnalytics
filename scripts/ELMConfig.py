#Recognized colors
RED = "Red"
GREEN = "Green"
BLUE = "Blue"
YELLOW = "Yellow"
BRIGHTFIELD = "Gray"
SKIP = "Skip"

UM_AREA = "Area (um^2)"

from ij.gui import Roi

import re

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

####
#
#  The Config Class - storing configuration info
#
####
class ConfigParams:
    numChannels = 4;
    numZ = 1;
    noZInFile = True;
    chanLabel = [SKIP, YELLOW, BLUE, BRIGHTFIELD];
    chansToSkip = [];
    inputDir = '';
    outputDir = '';
    # We need to avoid the scale bar in the bottom of the image, so set a roi that doesn't include it
    #analysisRoi = Roi(0,0,512,480)
    analysisRoi = Roi(0,0,1024,980)
    # Determines what index the dataset name is within the tokenized filename
    dsNameIdx = 4;
    pixelHeight = 1; # in micrometers
    pixelWidth = 1; # in micrometers
    wellNames = [] # List of well names to process, empty implies process all
    debugOutput =  False; # If true, additional info will be output
    
    def printCfg(self):
        print("Using Config:")
        print("\tinputDir:\t"    + self.inputDir)
        print("\toutputDir:\t"   + self.outputDir)
        if self.wellNames:
            print("\twellNames:\t" + ", ".join(self.wellNames))
        if self.chansToSkip:
            print("\tchansToSkip:\t" + ", ".join(self.chansToSkip))
        print("\tnumChannels:\t" + str(self.numChannels))
        print("\tnumZ:\t\t"        + str(self.numZ))
        print("\tnoZInFile:\t"   + str(self.noZInFile))
        print("\tchanLabel:\t"   + ", ".join(self.chanLabel))
        print("\tanalysisRoi:\t" + str(self.analysisRoi))
        print("\tpixelHeight:\t" + str(self.pixelHeight))
        print("\tpixelWidth:\t" + str(self.pixelWidth))
        print("\tdebugOutput:\t" + str(self.debugOutput))
        print("\n")