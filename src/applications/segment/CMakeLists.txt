ADD_EXECUTABLE(elmSegmentation
  elmSegmentation.cpp
)
TARGET_LINK_LIBRARIES(elmSegmentation core ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
