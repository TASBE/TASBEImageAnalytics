#!/bin/bash

checkFailure() {
    if [ ! $? -eq 0 ]
      then
        echo "Failed on $1"
        exit 1
    fi
}

FIJI_PATH="/home/nwalczak/workspace/fiji/Fiji.app"
CFG_PATH_NOESC="/media/nwalczak/XFS_Test/Resilio Sync/Resilio/2017_12_05 fine_stacks"
CFG_PATH="/home/nwalczak/workspace/elm/elm-git/testLN"

start=`date +%s`

export CLASSPATH=$PWD:$PWD/scripts:${CLASSPATH}
for cfgFile in "${CFG_PATH}"/*.ini
  do
    echo ${cfgFile}
    ${FIJI_PATH}/ImageJ-linux64 --heap 8192M --headless --run scripts/imagesToClouds.py cfgPath=\"${cfgFile}\"
    checkFailure "${cfgFile}"
done

end=`date +%s`
runtime=$((end-start))
echo "Runtime: ${runtime}"
