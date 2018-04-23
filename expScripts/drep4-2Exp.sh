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

## DAPG
echo "DAPG"

# 2018-04-02
echo "2018-04-02"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/replicoscillatorStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_04_02_CHO-Replicoscillator_Image_Data/DAPG/DAPG_04-02.ini\"
checkFailure "2018-04-02"

# 2018-04-03
echo "2018-04-03"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/replicoscillatorStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_04_02_CHO-Replicoscillator_Image_Data/DAPG/DAPG_04-03.ini\"
checkFailure "2018-04-03"

# 2018-04-04
echo "2018-04-04"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/replicoscillatorStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2018_04_02_CHO-Replicoscillator_Image_Data/DAPG/DAPG_04-04.ini\"
checkFailure "2018-04-04"




end=`date +%s`
runtime=$((end-start))
echo "Runtime: ${runtime}"
