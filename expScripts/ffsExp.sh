#!/bin/bash

checkFailure() {
    if [ ! $? -eq 0 ]
      then
        echo -e "${RED}Failed on $1${NC}"
        exit 1
    fi
}

FIJI_PATH="/home/nwalczak/workspace/fiji/Fiji.app"
CFG_PATH="/media/nwalczak/XFS_Test/ResilioSyncLN/Resilio/2018_10_04_FFS"
export CLASSPATH=$PWD:$PWD/scripts:${CLASSPATH}

RED='\033[0;31m'
NC='\033[0m' # No Color

start=`date +%s`

# Part 1
echo -e "${RED}Starting ffs_config${NC}"
${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/ffs_config.ini\"
checkFailure "ffs_config.ini"


end=`date +%s`
runtime=$((end-start))
echo -e "Runtime: ${runtime}"
