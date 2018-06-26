ADD_EXECUTABLE(elmSegmentation
  elmSegmentation.cpp
)
TARGET_LINK_LIBRARIES(elmSegmentation core ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

ADD_EXECUTABLE(synNotchStats
  synNotchStats.cpp
)
TARGET_LINK_LIBRARIES(synNotchStats core ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

ADD_EXECUTABLE(synNotchStatsNoSeg
  synNotchStatsNoSeg.cpp
)
TARGET_LINK_LIBRARIES(synNotchStatsNoSeg core ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})