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

#CMV_fov2
echo "CMV_fov2"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_02_01\ CHO-DREP_Image_Data/cmv_config.ini\"
checkFailure "CMV_fov2"

#TRE1_dox
echo "TRE1_dox"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_02_01\ CHO-DREP_Image_Data/TRE1_dox_config.ini\"
checkFailure "TRE1_dox"

#TRE1_no-dox
echo "RE1_no-dox"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_02_01\ CHO-DREP_Image_Data/TRE1_no-dox_config.ini\"
checkFailure "TRE1_no-dox"

#TRE2_dox
echo "TRE2_dox"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_02_01\ CHO-DREP_Image_Data/TRE2_dox_config.ini\"
checkFailure "TRE2_dox"

#TRE2_no-dox
echo "RE2_no-dox"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_02_01\ CHO-DREP_Image_Data/TRE2_no-dox_config.ini\"
checkFailure "TRE2_no-dox"

end=`date +%s`
runtime=$((end-start))
echo "Runtime: ${runtime}"
