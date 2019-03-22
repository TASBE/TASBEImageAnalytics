#!/bin/bash

checkFailure() {
    if [ ! $? -eq 0 ]
      then
        echo "Failed on $1"
        exit 1
    fi
}

FIJI_PATH="/home/nwalczak/workspace/fiji/Fiji.app"
CFG_PATH="/media/nwalczak/ExternalStorage/ResilioSyncLN/Resilio/2019_02_FIG_copy"

start=`date +%s`
export CLASSPATH=$PWD:$PWD/scripts:${CLASSPATH}

# FIG 1
#echo -e "${RED}Starting fig1_config${NC}"
#${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/fig1_config.ini\"
#checkFailure "fig1_config.ini"


# FIG 2
#echo -e "${RED}Starting fig2_config${NC}"
#${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/fig2_config.ini\"
#checkFailure "fig2_config.ini"


# FIG 3
#echo -e "${RED}Starting fig3_config${NC}"
#${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/fig3_config.ini\"
#checkFailure "fig3_config.ini"


# FIG 4
#echo -e "${RED}Starting fig4_config${NC}"
#${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/fig4_config.ini\"
#checkFailure "fig4_config.ini"


# FIG 5
#echo -e "${RED}Starting fig5_config${NC}"
#${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/fig5_config.ini\"
#checkFailure "fig5_config.ini"


# FIG 6 - defunct, replaced by day1_feb27
#echo -e "${RED}Starting fig6_config${NC}"
#${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/fig6_config.ini\"
#checkFailure "fig6_config.ini"

# FIG 6 day1_feb27
echo -e "${RED}Starting fig6_day1_feb27_config${NC}"
${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/fig6_day1_feb27_config.ini\"
checkFailure "fig6_day1_feb27_config.ini"

end=`date +%s`
runtime=$((end-start))
echo "Runtime: ${runtime}"
