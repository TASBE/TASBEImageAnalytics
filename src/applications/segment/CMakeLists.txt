ADD_EXECUTABLE(euclideanSegmentation
  euclideanSegmentation.cpp
)
TARGET_LINK_LIBRARIES(euclideanSegmentation ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
