#!/bin/bash

checkFailure() {
    if [ ! $? -eq 0 ]
      then
        echo "Failed on $1"
        exit 1
    fi
}

FIJI_PATH="/home/nwalczak/workspace/fiji/Fiji.app"
CFG_PATH="/media/nwalczak/ExternalStorage/ResilioSyncLN/Resilio/2019_03_PFS"

start=`date +%s`
export CLASSPATH=$PWD:$PWD/scripts:${CLASSPATH}

# PFS 1
echo -e "${RED}Starting pfs_031419_pfs4_config${NC}"
${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/pfs_031419_pfs4_config.ini\"
checkFailure "pfs_031419_pfs4_config.ini"


# PFS 2
echo -e "${RED}Starting pfs_031419_pfs5_config${NC}"
${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/pfs_031419_pfs5_config.ini\"
checkFailure "pfs_031419_pfs5_config.ini"


# PFS 3
echo -e "${RED}Starting pfs_031419_pfs6_config${NC}"
${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/pfs_031419_pfs6_config.ini\"
checkFailure "pfs_031419_pfs6_config.ini"


# PFS 4
echo -e "${RED}Starting pfs_031419_pfs7_config${NC}"
${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/cellStats.py cfgPath=\"${CFG_PATH}/pfs_031419_pfs7_config.ini\"
checkFailure "pfs_031419_pfs7_config.ini"


end=`date +%s`
runtime=$((end-start))
echo "Runtime: ${runtime}"
