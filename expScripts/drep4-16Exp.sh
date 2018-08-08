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

find /media/nwalczak/XFS_Test/ResilioSyncLN/Resilio/2018_04_16_CHO-Replicoscillator_Image_Data -name \*.ini &> /tmp/inis

while read line;
  do
    ${FIJI_PATH}/ImageJ-linux64 --ij2 --run scripts/replicoscillatorStats.py cfgPath=\"${line}\"
    checkFailure $(basename $line)
done < /tmp/inis


end=`date +%s`
runtime=$((end-start))
echo "Runtime: ${runtime}"
