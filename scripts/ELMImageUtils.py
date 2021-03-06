# Copyright (C) 2011 - 2019, Raytheon BBN Technologies and contributors listed
# in the AUTHORS file in TASBE Flow Analytics distribution's top directory.
#
# This file is part of the TASBE Flow Analytics package, and is distributed
# under the terms of the GNU General Public License, with a linking
# exception, as described in the file LICENSE in the TASBE Image Analysis
# package distribution's top directory.

from ij.process import ImageConverter, ImageProcessor, ByteProcessor
from ij.plugin import ChannelSplitter, ImageCalculator
from ij import IJ, ImagePlus, WindowManager

from java.awt import Color

import os

import ELMConfig

def getGrayScaleImage(currIP, c, chanName, cfg):
    if (cfg.hasValue(ELMConfig.upperLeftExclusionX)):
        ulExclusionX = cfg.getValue(ELMConfig.upperLeftExclusionX)
    else:
        ulExclusionX = 0

    if (cfg.hasValue(ELMConfig.upperLeftExclusionY)):
        ulExclusionY = cfg.getValue(ELMConfig.upperLeftExclusionY)
    else:
        ulExclusionY = 0

    if (cfg.hasValue(ELMConfig.lowerRightExclusionX)):
        lrExclusionX = cfg.getValue(ELMConfig.lowerRightExclusionX)
    else:
        lrExclusionX = currIP.getWidth()

    if (cfg.hasValue(ELMConfig.lowerRightExclusionY)):
        lrExclusionY = cfg.getValue(ELMConfig.lowerRightExclusionY)
    else:
        lrExclusionY = currIP.getHeight()
    
    imgType = currIP.getType()
    if (chanName in cfg.getValue(ELMConfig.chansToSkip)): # Don't process skip channels
        currIP.close()
        return None
    elif imgType == ImagePlus.COLOR_RGB or imgType == ImagePlus.COLOR_256:
        if (chanName == ELMConfig.BRIGHTFIELD):
            toGray = ImageConverter(currIP)
            toGray.convertToGray8()
        elif (chanName == ELMConfig.BLUE) \
                or (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.RED) \
                or (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.GREEN): #
            chanIdx = 2
            if (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.RED):
                chanIdx = 0
            elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.GREEN):
                chanIdx = 1;
            imgChanns = ChannelSplitter.split(currIP);
            currIP.close()
            currIP = imgChanns[chanIdx];
    
            # Clear the Exclusion zone, so it doesn't mess with  thresholding
            imgProc = currIP.getProcessor();
            imgProc.setColor(Color(0,0,0))
            imgProc.fillRect(lrExclusionX, lrExclusionY, currIP.getWidth(), currIP.getHeight())
            imgProc.fillRect(0, 0, ulExclusionX, ulExclusionY)
        elif (chanName == ELMConfig.YELLOW):
            # Clear the Exclusion zone, so it doesn't mess with  thresholding
            imgProc = currIP.getProcessor();
            imgProc.setColor(Color(0,0,0))
            imgProc.fillRect(lrExclusionX, lrExclusionY, currIP.getWidth(), currIP.getHeight())
            imgProc.fillRect(0, 0, ulExclusionX, ulExclusionY)
    
            # Create a new image that consists of the average of the red & green channels
            title = currIP.getTitle()
            width = currIP.getWidth();
            height = currIP.getHeight();
            newPix = ByteProcessor(width, height)
            for x in range(0, width) :
                for y in range(0,height) :
                    currPix = currIP.getPixel(x,y);
                    newPix.putPixel(x, y, (currPix[0] + currPix[1]) / 2)
            currIP.close()
            currIP = ImagePlus(title, newPix)
        else:
            print "ERROR: Unrecognized channel name! Name: " + chanName
            currIP.close()
            return None
    elif imgType == ImagePlus.GRAY16 or imgType == ImagePlus.GRAY32 or imgType == ImagePlus.GRAY8:
        if not imgType == ImagePlus.GRAY8:
            toGray = ImageConverter(currIP)
            toGray.convertToGray8()
    else:
        print "ERROR: Unrecognized channel name & image type! Channel: " + chanName + ", imgType: " + str(imgType)
        currIP.close()
        return None

    return currIP
            

###
#
#
###
def getThresholdedMask(currIP, c, z, t, chanName, cfg, wellPath, dbgOutDesc):
    if (cfg.hasValue(ELMConfig.upperLeftExclusionX)):
        ulExclusionX = cfg.getValue(ELMConfig.upperLeftExclusionX)
    else:
        ulExclusionX = 0

    if (cfg.hasValue(ELMConfig.upperLeftExclusionY)):
        ulExclusionY = cfg.getValue(ELMConfig.upperLeftExclusionY)
    else:
        ulExclusionY = 0

    if (cfg.hasValue(ELMConfig.lowerRightExclusionX)):
        lrExclusionX = cfg.getValue(ELMConfig.lowerRightExclusionX)
    else:
        lrExclusionX = currIP.getWidth()

    if (cfg.hasValue(ELMConfig.lowerRightExclusionY)):
        lrExclusionY = cfg.getValue(ELMConfig.lowerRightExclusionY)
    else:
        lrExclusionY = currIP.getHeight()
    
    imgType = currIP.getType()
    if (chanName in cfg.getValue(ELMConfig.chansToSkip)): # Don't process skip channels
        currIP.close()
        return None
    elif imgType == ImagePlus.COLOR_RGB or imgType == ImagePlus.COLOR_256:
        if (chanName == ELMConfig.BRIGHTFIELD):
            toGray = ImageConverter(currIP)
            toGray.convertToGray8()
            if cfg.params[ELMConfig.imgType] == "png":
                darkBackground = True
            else:
                darkBackground = False
        elif (chanName == ELMConfig.BLUE) \
                or (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.RED) \
                or (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.GREEN): #
            chanIdx = 2
            if (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.RED):
                chanIdx = 0
            elif (cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.GREEN):
                chanIdx = 1;
            imgChanns = ChannelSplitter.split(currIP);
            currIP.close()
            currIP = imgChanns[chanIdx];
    
            # Clear the Exclusion zone, so it doesn't mess with  thresholding
            imgProc = currIP.getProcessor();
            imgProc.setColor(Color(0,0,0))
            imgProc.fillRect(lrExclusionX, lrExclusionY, currIP.getWidth(), currIP.getHeight())
            imgProc.fillRect(0, 0, ulExclusionX, ulExclusionY)
            darkBackground = True
        elif (chanName == ELMConfig.YELLOW):
            # Clear the Exclusion zone, so it doesn't mess with  thresholding
            imgProc = currIP.getProcessor();
            imgProc.setColor(Color(0,0,0))
            imgProc.fillRect(lrExclusionX, lrExclusionY, currIP.getWidth(), currIP.getHeight())
            imgProc.fillRect(0, 0, ulExclusionX, ulExclusionY)
    
            # Create a new image that consists of the average of the red & green channels
            title = currIP.getTitle()
            width = currIP.getWidth();
            height = currIP.getHeight();
            newPix = ByteProcessor(width, height)
            for x in range(0, width) :
                for y in range(0,height) :
                    currPix = currIP.getPixel(x,y);
                    newPix.putPixel(x, y, (currPix[0] + currPix[1]) / 2)
            currIP.close()
            currIP = ImagePlus(title, newPix)
            darkBackground = True
        else:
            print "ERROR: Unrecognized channel name! Name: " + chanName
            currIP.close()
            return None
    elif imgType == ImagePlus.GRAY16 or imgType == ImagePlus.GRAY32 or imgType == ImagePlus.GRAY8:
        if (chanName == ELMConfig.BRIGHTFIELD):
            if cfg.params[ELMConfig.imgType] == "png":
                darkBackground = True
            else:
                darkBackground = False
        else:
            darkBackground = True
        
        if not imgType == ImagePlus.GRAY8: 
            toGray = ImageConverter(currIP)
            toGray.convertToGray8()
    else:
        print "ERROR: Unrecognized channel name & image type! Channel: " + chanName + ", imgType: " + str(imgType)
        currIP.close()
        return None

    WindowManager.setTempCurrentImage(currIP);

    if cfg.getValue(ELMConfig.debugOutput):
        IJ.saveAs('png', os.path.join(wellPath, "Processing_" + dbgOutDesc + ".png"))

    upperThreshImg = currIP.duplicate()

    # If threshold value is set, use it
    if (cfg.hasValue(ELMConfig.imageThreshold)):
        thresh = cfg.getValue(ELMConfig.imageThreshold)
        if (darkBackground):
            currIP.getProcessor().setThreshold(thresh, 255, ImageProcessor.NO_LUT_UPDATE)
        else:
            currIP.getProcessor().setThreshold(0, thresh, ImageProcessor.NO_LUT_UPDATE)
    else: # Otherise, automatically compute threshold
        threshMethod = "Default"
        if cfg.hasValue(ELMConfig.thresholdMethod):
            threshMethod = cfg.getValue(ELMConfig.thresholdMethod)
    
        currIP.getProcessor().setAutoThreshold(threshMethod, darkBackground, ImageProcessor.NO_LUT_UPDATE)
        threshRange = currIP.getProcessor().getMaxThreshold() - currIP.getProcessor().getMinThreshold()
        #print "\t\tZ = " + str(z) + ", T = " + str(t) +  ", chan " + chanName + ": Using default threshold of minThresh: " + str(currIP.getProcessor().getMinThreshold()) + ", maxThresh: " + str(currIP.getProcessor().getMaxThreshold())
        if currIP.getType() != ImagePlus.GRAY8:
            print "\tChannel " + chanName + " is not GRAY8, instead type is %d" % currIP.getType()
        if threshRange > cfg.getValue(ELMConfig.maxThreshRange):
            if (cfg.hasValue(ELMConfig.defaultThreshold)):
                thresh = cfg.getValue(ELMConfig.defaultThreshold)
                print "\t\tZ = " + str(z) + ", T = " + str(t) +  ", chan " + chanName + ": Using default threshold of " + str(thresh) + ", minThresh: " + str(currIP.getProcessor().getMinThreshold()) + ", maxThresh: " + str(currIP.getProcessor().getMaxThreshold())
                if (darkBackground):
                    currIP.getProcessor().setThreshold(thresh, 255, ImageProcessor.NO_LUT_UPDATE)
                else:
                    currIP.getProcessor().setThreshold(0, thresh, ImageProcessor.NO_LUT_UPDATE)  
            else:
                print "\t\tZ = " + str(z) + ", T = " + str(t) +  ", chan " + chanName + ": Ignored Objects due to threshold range! minThresh: " + str(currIP.getProcessor().getMinThreshold()) + ", maxThresh: " + str(currIP.getProcessor().getMaxThreshold())
                currIP.close()
                return None

    IJ.run(currIP, "Convert to Mask", "")
    
    # Clear out exclusion zones
    imgProc = currIP.getProcessor();
    imgProc.fillRect(lrExclusionX, lrExclusionY, currIP.getWidth(), currIP.getHeight())
    imgProc.fillRect(0, 0, ulExclusionX, ulExclusionY)
    
    IJ.run(currIP, "Close-", "")
    
    # Brightfield has an additional thresholding step
    if cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.BRIGHTFIELD:
        if cfg.getValue(ELMConfig.debugOutput):
            IJ.saveAs('png', os.path.join(wellPath, "OrigMask_" + dbgOutDesc + ".png"))

        upperThresh = 255 * 0.95
        upperThreshImg.getProcessor().setThreshold(upperThresh, 255, ImageProcessor.NO_LUT_UPDATE)
        IJ.run(upperThreshImg, "Convert to Mask", "")
        IJ.run(upperThreshImg, "Close-", "")
        if cfg.getValue(ELMConfig.debugOutput):
            WindowManager.setTempCurrentImage(upperThreshImg);
            IJ.saveAs('png', os.path.join(wellPath, "UpperThreshMask_" + dbgOutDesc + ".png"))

        ic = ImageCalculator()
        compositeMask = ic.run("OR create", currIP, upperThreshImg)
        IJ.run(compositeMask, "Close-", "")
        currIP.close()
        currIP = compositeMask
        WindowManager.setTempCurrentImage(currIP);
        
    if cfg.getValue(ELMConfig.debugOutput):
            WindowManager.setTempCurrentImage(currIP);
            IJ.saveAs('png', os.path.join(wellPath, "Binary_" + dbgOutDesc + ".png"))
    
    upperThreshImg.close()
    return currIP