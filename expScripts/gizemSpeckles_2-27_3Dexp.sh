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
VIZ_PATH="/home/nwalczak/workspace/elm/elm-python/src/synNotchStatsPlots.py"
CFG_PATH="/media/nwalczak/XFS_Test/Resilio Sync/Resilio/2018_02_27_Gizem_speckles1/gizemSpeckles_config.ini"

start=`date +%s`
export CLASSPATH=$PWD:$PWD/scripts:${CLASSPATH}

# 3D Cell Stats
#${FIJI_PATH}/ImageJ-linux64 --heap 10240M --headless --run scripts/3DcellStats.py cfgPath=\"${CFG_PATH}\"
#checkFailure "${CFG_PATH}"

DIST_TYPE[0]=avg
DIST_TYPE[1]=nn
DIST_TYPE[2]=exp

declare -A RECVR_THRESH=( [avg]=6 [nn]=6 [exp]=.25 )

# SynNotchStats
OUTPUT_DIR1=$(grep outputDir "${CFG_PATH}" | cut -d= -f2)
# Remove leading white space, or else problems
OUTPUT_DIR2="$(echo -e "${OUTPUT_DIR1}" | sed -e 's/^[[:space:]]*//')"

for DT in ${DIST_TYPE[@]};
  do
    echo "Processing distType: ${DT}"
    for wellPath in "${OUTPUT_DIR2}"/*/
      do
        echo "    `basename "${wellPath}"`"
        ${ELM_PATH}/synNotchStatsNoSeg "${wellPath}"/Red_segParams.ini "${wellPath}"/Green_segParams.ini "${wellPath}"/Blue_segParams.ini ${DT} 1> /dev/null
        if [ -f "${wellPath}/SynNotchNoSegOutput/distances.csv" ];
          then
            ${VIZ_PATH} "${wellPath}/SynNotchNoSegOutput/distances.csv" "${wellPath}/SynNotchNoSegOutput" ${DT} ${RECVR_THRESH[${DT}]}
          else
            echo "        distances csv doesn't exist!"
        fi
    done
    
    # Process all data    
    find "${OUTPUT_DIR2}" -name distances.csv -exec cat '{}' \; &> "${OUTPUT_DIR2}"/all_dists_${DT}.csv

    echo "    Visualize all data, DT: ${DIST_TYPE[0]}" 
    ${VIZ_PATH} "${OUTPUT_DIR2}/all_dists_${DT}.csv" "${OUTPUT_DIR2}" ${DT} ${RECVR_THRESH[${DT}]}

done



end=`date +%s`
runtime=$((end-start))
echo "Runtime: ${runtime}"
