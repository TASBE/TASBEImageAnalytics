# TASBE Image Analytics

TASBE Image Analytics is an image processing pipeline leveraging ImageJ for analysis of microscopy images.
It was designed for detecting and tracking cells in fluorescent images for the purposes of quantify the shape of cell balls.
It also contains some experimental code for generating point clouds from fluorescent microscopy images.
It can also utilize the TrackMate plugin for tracking the cell ball detections.

The major functionality is encapsulated in three scripts:
```
cellStats.py
cellStatsTracking.py
3Dcellstats.py
```
The first script is able to process a microscopy experiment and detect cell clusters using an image thresholding-based approach and outputs statistics about the detected cell clusters in a CSV file.  The `cellStatsTracking` uses the same threshold-based approach for detection but then is able to track the cells over time and output information about the clusters and their tracks.  The last script is can take Z-slices and turn the data into a point cloud file for 3d visualization.  A threshold on the intensity of fluorescence can be used to filter out points.

Examples of how these scripts can be found in [TASBEImageAnalytics-Tutorial](https://github.com/TASBE/TASBEImageAnalytics-Tutorial).

The point cloud creation process utilizes parts of the PCL library and requires source code to be build in C++.  The following commands can be used to build the C++ code if the propery dependencies are resolved:
```
mkdir build
cd build
cmake ..
make
```
This requires CMake to be installed along with PCL.

