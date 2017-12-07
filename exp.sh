#!/bin/bash

FIJI_PATH=/home/nwalczak/workspace/fiji/Fiji.app

start=`date +%s`

#cholka_dots_dox
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/cholka_dots_dox_gradient/plate2/plate2Config.ini\"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/cholka_dots_dox_gradient/plate3/plate3Config.ini\"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/cholka_dots_dox_gradient/plate4/plate4Config.ini\"

#polka_dots_repeat
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/polka_dots_repeat/plate1Config.ini\"
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/polka_dots_repeat/plate3Config.ini\"

#total_size_dependence
${FIJI_PATH}/ImageJ-linux64 --ij2 --headless --run scripts/cellStats.py cfgPath=\"/media/nwalczak/XFS_Test/Resilio\ Sync/Resilio/total_size_dependence_experiment/total_size_cfg.ini\"

end=`date +%s`
runtime=$((end-start))
echo "Runtime: ${runtime}"
