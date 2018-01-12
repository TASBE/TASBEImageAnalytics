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

#cholka_dots_dox
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2017_09_20\ cholka_dots_dox_gradient/plate2/plate2Config.ini\"
checkFailure "cholka_dots_dox_gradient plate2"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2017_09_20\ cholka_dots_dox_gradient/plate3/plate3Config.ini\"
checkFailure "holka_dots_dox_gradient plate3"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2017_09_20\ cholka_dots_dox_gradient/plate4/plate4Config.ini\"
checkFailure "cholka_dots_dox_gradient plate4"

#polka_dots_repeat
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2017_08_28\ polka_dots_repeat/plate1Config.ini\"
checkFailure "polka_dots_repeat plate1"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2017_08_28\ polka_dots_repeat/plate3Config.ini\"
checkFailure "polka_dots_repeat plate3"

#total_size_dependence
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/2017_12_05\ total_size_dependence_experiment/total_size_cfg.ini\"
checkFailure "total_size_dependence_experiment"

end=`date +%s`
runtime=$((end-start))
echo "Runtime: ${runtime}"
