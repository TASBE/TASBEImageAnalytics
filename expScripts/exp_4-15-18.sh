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

# 28hrs_Apr14_2018_9pm
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/ResilioSyncLN/Resilio/2018_04_13_EXP_5050replicates/EXP_2018_04_15/28hrs_Apr14_2018_9pm/28hrs_Apr14_2018_9pm_config.ini\"
checkFailure "28hrs_Apr14_2018_9pm"
# 44hrs_Apr15_2018
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/ResilioSyncLN/Resilio/2018_04_13_EXP_5050replicates/EXP_2018_04_15/44hrs_Apr15_2018/44hrs_Apr15_2018_config.ini\"
checkFailure "44hrs_Apr15_2018"
# Apr20_2018_11am
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/ResilioSyncLN/Resilio/2018_04_13_EXP_5050replicates/EXP_2018_04_15/Apr20_2018_11am/Apr20_2018_11am_config.ini\"
checkFailure "Apr20_2018_11am"

end=`date +%s`
runtime=$((end-start))
echo "Runtime: ${runtime}"
