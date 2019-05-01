/*
  Copyright (C) 2011 - 2019, Raytheon BBN Technologies and contributors listed
  in the AUTHORS file in TASBE Flow Analytics distribution's top directory.

  This file is part of the TASBE Flow Analytics package, and is distributed
  under the terms of the GNU General Public License, with a linking
  exception, as described in the file LICENSE in the TASBE Image Analysis
  package distribution's top directory.
*/

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;
using namespace pcl::visualization;
using namespace std;

PCLVisualizer viewer("Cloud Viewer");

bool done = false;
bool nextStep = false;

void keyHandler(const visualization::KeyboardEvent &e, void *)
{
    if (!e.keyDown()) {
        return;
    }
    const char keyPress = e.getKeyCode();
    // Check for special symbolic key
    if (keyPress == '\0') {
        string keySym = e.getKeySym();
        if (keySym == "Up") {
        }
    } else if (keyPress == 27) { // ESC is pressed.
        done = true;
        nextStep = true;
    } else if ((keyPress == ' ')) {
        nextStep = true;
    }

}

void printUsage() {
    cout
            << "This program is designed to read in point clouds from 3dtk "
            << "files and apply filters to those clouds and then write them "
            << "back."
            << "Usage: "
            << endl
            << "<3dtk_dataset_dir> <start_frame> <end_frame>"
            << endl;

}

int main(const int argc, const char **argv) {

    if (argc < 4) {
        printUsage();
        return 0;
    }

    int startFrame = atoi(argv[2]);
    int endFrame = atoi(argv[3]);
    char buf[128];
    snprintf(buf,128,"Frame %d/%d", startFrame, endFrame);
    string basePath = argv[1];
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(1.5,"coordSys",0);
    viewer.addText(buf,0,10,22,1,1,1,"frames");
    viewer.registerKeyboardCallback(keyHandler);

    for (int fr = startFrame; fr < endFrame && !done; fr++) {

        snprintf(buf,128,"frame%d.pcd",fr);
        string pcloudPath = basePath + "/" + buf;
        PointCloud<PointXYZI>::Ptr cloud (new PointCloud<PointXYZI>);
        io::loadPCDFile (pcloudPath.c_str(), *cloud);

        //PointCloudColorHandlerGenericField<PointXYZI> colorHandler(cloud,
        //        "intensity");
        // Add pointcloud
        snprintf(buf, 128, "cloud%03d", fr);
        //viewer.addPointCloud<PointXYZI>(cloud, colorHandler, buf);

        if (fr > startFrame) {
            snprintf(buf, 128, "cloud%03d", fr - 1);
            viewer.removePointCloud(buf);
        }

        // Update Frame count display
        snprintf(buf,128,"Frame %d/%d", fr, endFrame);
        viewer.updateText(buf,0,10,22,1,1,1,"frames");

        // Spin display loop until nextStep is signaled
        while (!nextStep) {
            viewer.spinOnce(100);
        }
        nextStep = false;
    }
    viewer.close();
}




