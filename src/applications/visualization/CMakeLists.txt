ADD_EXECUTABLE (pcdViewer 
   pcdViewer.cpp)

ADD_EXECUTABLE(simple_hdlviewer
  SampleViewer.cpp
)

TARGET_LINK_LIBRARIES (pcdViewer ${PCL_LIBRARIES})
  
TARGET_LINK_LIBRARIES(simple_hdlviewer ${PCL_LIBRARIES})
