from ij.process import ImageConverter, ImageProcessor, ByteProcessor
from ij.plugin import ChannelSplitter, ImageCalculator
from ij import IJ, ImagePlus, WindowManager

from java.awt import Color

import os

import ELMConfig


def getGrayScaleImage(currIP, c, z, zStr, chanStr, chanName, cfg, wellPath, wellName):
    roiX = cfg.getValue(ELMConfig.pcloudExclusionX)
    roiY = cfg.getValue(ELMConfig.pcloudExclusionY)
    if (chanName == ELMConfig.BRIGHTFIELD):
        # Clear the Exclusion zone, so it doesn't mess with  thresholding
        imgProc = currIP.getProcessor();
        imgProc.setColor(Color(128,128,128))
        imgProc.fillRect(roiX, roiY, currIP.getWidth(), currIP.getHeight())

        toGray = ImageConverter(currIP)
        toGray.convertToGray8()
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
        imgProc.fillRect(roiX, roiY, currIP.getWidth(), currIP.getHeight())
        darkBackground = True
    elif (chanName == ELMConfig.YELLOW):
        # Clear the Exclusion zone, so it doesn't mess with  thresholding
        imgProc = currIP.getProcessor();
        imgProc.setColor(Color(0,0,0))
        imgProc.fillRect(roiX, roiY, currIP.getWidth(), currIP.getHeight())

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
    elif (chanName == ELMConfig.SKIP):
        currIP.close()
        return None
    
    WindowManager.setTempCurrentImage(currIP);

    if cfg.getValue(ELMConfig.debugOutput):
        IJ.saveAs('png', os.path.join(wellPath, "Processing_" + wellName + "_" + zStr + "_" + chanStr + ".png"))

    upperThreshImg = currIP.duplicate()

    currIP.getProcessor().setAutoThreshold("Default", darkBackground, ImageProcessor.NO_LUT_UPDATE)
    threshRange = currIP.getProcessor().getMaxThreshold() - currIP.getProcessor().getMinThreshold()
    if currIP.getType() != ImagePlus.GRAY8 :
        print "\tChannel " + chanName + " is not GRAY8, instead type is %d" % currIP.getType()
    if threshRange > 230:
        print "\t\tZ = " + str(z) + " chan " + chanName + ": Ignored Objects due to threshold range!"
        currIP.close()
        return None

    IJ.run(currIP, "Convert to Mask", "")
    IJ.run(currIP, "Close-", "")
    
    # Brightfield has an additional thresholding step
    if cfg.getValue(ELMConfig.chanLabel)[c] == ELMConfig.BRIGHTFIELD:
        if cfg.getValue(ELMConfig.debugOutput):
            IJ.saveAs('png', os.path.join(wellPath, "OrigMask" + wellName + "_" + zStr + "_" + chanStr + ".png"))

        upperThresh = 255 * 0.95
        upperThreshImg.getProcessor().setThreshold(upperThresh, 255, ImageProcessor.NO_LUT_UPDATE)
        IJ.run(upperThreshImg, "Convert to Mask", "")
        IJ.run(upperThreshImg, "Close-", "")
        if cfg.getValue(ELMConfig.debugOutput):
            WindowManager.setTempCurrentImage(upperThreshImg);
            IJ.saveAs('png', os.path.join(wellPath, "UpperThreshMask" + wellName + "_" + zStr + "_" + chanStr + ".png"))

        ic = ImageCalculator()
        compositeMask = ic.run("OR create", currIP, upperThreshImg)
        IJ.run(compositeMask, "Close-", "")
        currIP.close()
        currIP = compositeMask
        WindowManager.setTempCurrentImage(currIP);
        
    if cfg.getValue(ELMConfig.debugOutput):
            WindowManager.setTempCurrentImage(currIP);
            IJ.saveAs('png', os.path.join(wellPath, "Binary_" + wellName + "_" + zStr + "_" + chanStr + ".png"))
    
    upperThreshImg.close()
    return currIP