#!/bin/bash

checkFailure() {
    if [ ! $? -eq 0 ]
      then
        echo "Failed on $1"
        exit 1
    fi
}

FIJI_PATH="/home/nwalczak/workspace/fiji/Fiji.app"
ELM_PATH="/home/nwalczak/workspace/elm/elm-git/build/bin/applications/segment"
CFG_PATH="/media/nwalczak/XFS_Test/Resilio Sync/Resilio/2018_02_27_Gizem_speckles1/gizemSpeckles_config.ini"

start=`date +%s`
export CLASSPATH=$PWD:$PWD/scripts:${CLASSPATH}

# 3D Cell Stats
#${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/3DcellStats.py cfgPath=\"${CFG_PATH}\"
#checkFailure "${CFG_PATH}"

# SynNotchStats
OUTPUT_DIR1=$(grep outputDir "${CFG_PATH}" | cut -d= -f2)
# Remove leading white space, or else problems
OUTPUT_DIR2="$(echo -e "${OUTPUT_DIR1}" | sed -e 's/^[[:space:]]*//')"
for wellPath in "${OUTPUT_DIR2}"/*/
  do
    ${ELM_PATH}/synNotchStats "${wellPath}"/Red_segParams.ini "${wellPath}"/Green_segParams.ini "${wellPath}"/Blue_segParams.ini
done

end=`date +%s`
runtime=$((end-start))
echo "Runtime: ${runtime}"
