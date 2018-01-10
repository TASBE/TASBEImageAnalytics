ADD_EXECUTABLE(euclideanSegmentation
  euclideanSegmentation.cpp
)
TARGET_LINK_LIBRARIES(euclideanSegmentation core ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
