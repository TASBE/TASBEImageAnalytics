#!/bin/bash

checkFailure() {
    if [ ! $? -eq 0 ]
      then
        echo "Failed on $1"
        exit 1
    fi
}

FIJI_PATH=/home/nwalczak/workspace/fiji/Fiji.app

start=`date +%s`

export CLASSPATH=$PWD:$PWD/scripts:${CLASSPATH}

## March 1-2
echo "March 1-2"

#CMV_fov1
echo "CMV_fov1"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_03_05_CHO-DREP_Image_Data/2018_March_1-2/cmv_fov1_config.ini\"
checkFailure "CMV_fov1"

#CMV_fov2
echo "CMV_fov2"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_03_05_CHO-DREP_Image_Data/2018_March_1-2/cmv_fov2_config.ini\"
checkFailure "CMV_fov2"

#TRE1_fov1
echo "TRE1_fov1"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_03_05_CHO-DREP_Image_Data/2018_March_1-2/TRE1_fov1_config.ini\"
checkFailure "TRE1_fov1"

#TRE1_fov2
echo "TRE1_fov2"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_03_05_CHO-DREP_Image_Data/2018_March_1-2/TRE1_fov2_config.ini\"
checkFailure "TRE1_fov2"

#TRE2_fov1
echo "TRE2_fov1"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_03_05_CHO-DREP_Image_Data/2018_March_1-2/TRE2_fov1_config.ini\"
checkFailure "TRE2_fov1"

#TRE2_fov2
echo "TRE2_fov2"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_03_05_CHO-DREP_Image_Data/2018_March_1-2/TRE2_fov2_config.ini\"
checkFailure "TRE2_fov2"

## March 3-4
echo "March 3-4"

#CMV_fov1
echo "CMV_fov1"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_03_05_CHO-DREP_Image_Data/2018_March_3-4/cmv_fov1_config.ini\"
checkFailure "CMV_fov1"

#CMV_fov2
echo "CMV_fov2"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_03_05_CHO-DREP_Image_Data/2018_March_3-4/cmv_fov2_config.ini\"
checkFailure "CMV_fov2"

#TRE1_fov1
echo "TRE1_fov1"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_03_05_CHO-DREP_Image_Data/2018_March_3-4/TRE1_fov1_config.ini\"
checkFailure "TRE1_fov1"

#TRE1_fov2
echo "TRE1_fov2"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_03_05_CHO-DREP_Image_Data/2018_March_3-4/TRE1_fov2_config.ini\"
checkFailure "TRE1_fov2"

#TRE2_fov1
echo "TRE2_fov1"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_03_05_CHO-DREP_Image_Data/2018_March_3-4/TRE2_fov1_config.ini\"
checkFailure "TRE2_fov1"

#TRE2_fov2
echo "TRE2_fov2"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_03_05_CHO-DREP_Image_Data/2018_March_3-4/TRE2_fov2_config.ini\"
checkFailure "TRE2_fov2"

end=`date +%s`
runtime=$((end-start))
echo "Runtime: ${runtime}"
