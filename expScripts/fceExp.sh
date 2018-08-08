#!/bin/bash

checkFailure() {
    if [ ! $? -eq 0 ]
      then
        echo "Failed on $1"
        exit 1
    fi
}

FIJI_PATH="/home/nwalczak/workspace/fiji/Fiji.app"
CFG_PATH="/media/nwalczak/XFS_Test/ResilioSyncLN/Resilio/2018_07_23_FCE"
export CLASSPATH=$PWD:$PWD/scripts:${CLASSPATH}

start=`date +%s`

#${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/part1_config.ini\"
#checkFailure "part1_config.ini"

${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/part2_config.ini\"
checkFailure "part2_config.ini"

end=`date +%s`
runtime=$((end-start))
echo "Runtime: ${runtime}"
