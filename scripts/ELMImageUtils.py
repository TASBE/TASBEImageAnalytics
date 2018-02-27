from ij.process import ImageConverter, ImageProcessor, ByteProcessor
from ij.plugin import ChannelSplitter, ImageCalculator
from ij import IJ, ImagePlus, WindowManager

from java.awt import Color

import os

import ELMConfig


def getGrayScaleImage(currIP, c, z, t, chanName, cfg, wellPath, dbgOutDesc):
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
            # Clear the Exclusion zone, so it doesn't mess with  thresholding
            imgProc = currIP.getProcessor();
            imgProc.setColor(Color(128,128,128))
            imgProc.fillRect(lrExclusionX, lrExclusionY, currIP.getWidth(), currIP.getHeight())
            imgProc.fillRect(0, 0, ulExclusionX, ulExclusionY)
    
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
            fillColor = Color(128,128,128)
        else:
            darkBackground = True
            fillColor = Color(0,0,0)
        # Clear the Exclusion zone, so it doesn't mess with  thresholding
        imgProc = currIP.getProcessor();
        imgProc.setColor(fillColor)
        imgProc.fillRect(lrExclusionX, lrExclusionY, currIP.getWidth(), currIP.getHeight())
        imgProc.fillRect(0, 0, ulExclusionX, ulExclusionY)
        
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

    currIP.getProcessor().setAutoThreshold("Default", darkBackground, ImageProcessor.NO_LUT_UPDATE)
    threshRange = currIP.getProcessor().getMaxThreshold() - currIP.getProcessor().getMinThreshold()
    if currIP.getType() != ImagePlus.GRAY8:
        print "\tChannel " + chanName + " is not GRAY8, instead type is %d" % currIP.getType()
    if threshRange > 230:
        print "\t\tZ = " + str(z) + ", T = " + str(t) +  ", chan " + chanName + ": Ignored Objects due to threshold range!"
        currIP.close()
        return None

    IJ.run(currIP, "Convert to Mask", "")
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