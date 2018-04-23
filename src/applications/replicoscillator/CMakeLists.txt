ADD_EXECUTABLE(edgedBasedDetection
  edgedBasedDetection.cpp
)
TARGET_LINK_LIBRARIES(edgedBasedDetection core ${OpenCV_LIBRARIES})

ADD_EXECUTABLE(smoothingFilter
  smoothingFilter.cpp
)
TARGET_LINK_LIBRARIES(smoothingFilter core ${OpenCV_LIBRARIES})