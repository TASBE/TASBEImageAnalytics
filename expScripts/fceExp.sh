#!/bin/bash

checkFailure() {
    if [ ! $? -eq 0 ]
      then
        echo -e "${RED}Failed on $1${NC}"
        exit 1
    fi
}

FIJI_PATH="/home/nwalczak/workspace/fiji/Fiji.app"
CFG_PATH="/media/nwalczak/XFS_Test/ResilioSyncLN/Resilio/2018_07_23_FCE"
export CLASSPATH=$PWD:$PWD/scripts:${CLASSPATH}

RED='\033[0;31m'
NC='\033[0m' # No Color

start=`date +%s`

# Part 1
echo -e "${RED}Starting part1_config${NC}"
${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/part1_config.ini\"
checkFailure "part1_config.ini"

echo -e "${RED}Starting part1_noCHO_config${NC}"
${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/part1_noCHO_config.ini\"
checkFailure "part1_noCHO_config.ini"

echo -e "${RED}Starting part1_Pos021_config${NC}"
${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/part1_Pos021_config.ini\"
checkFailure "part1_Pos021_config.ini"


# Part 2
echo -e "${RED}Starting part2_config${NC}"
${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/part2_config.ini\"
checkFailure "part2_config.ini"

echo -e "${RED}Starting part2_noCHO_config${NC}"
${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/part2_noCHO_config.ini\"
checkFailure "part2_noCHO_config.ini"

echo -e "${RED}Starting part2_Pos021_config${NC}"
${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/part2_Pos021_config.ini\"
checkFailure "part2_Pos021_config.ini"

echo -e "${RED}Starting part2_2Col_config${NC}"
${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/part2_2Col_config.ini\"
checkFailure "part2_2Col_config.ini"

echo -e "${RED}Starting part2_3Col_config${NC}"
${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/part2_3Col_config.ini\"
checkFailure "part2_3Col_config.ini"

end=`date +%s`
runtime=$((end-start))
echo -e "Runtime: ${runtime}"
