#!/bin/bash

checkFailure() {
    if [ ! $? -eq 0 ]
      then
        echo "Failed on $1"
        exit 1
    fi
}

ELM_PATH="/home/nwalczak/workspace/elm/elm-git/build/bin/applications/segment"

# SynNotchStats
start=`date +%s`
OUTPUT_DIR2="/media/nwalczak/XFS_Test/ResilioSyncLN/Resilio/2018_03_28_Gizem_speckles2/Z stacks/processing_outputs"
for wellPath in "${OUTPUT_DIR2}"/*/
  do
    echo "Processing ${wellPath}"
    ${ELM_PATH}/synNotchStats "${wellPath}"/Red_segParams.ini "${wellPath}"/Green_segParams.ini "${wellPath}"/Blue_segParams.ini
    checkFailure
    echo "\n\n\n"
done

end=`date +%s`
runtime=$((end-start))
echo "Runtime: ${runtime}"
