ADD_LIBRARY(core SHARED  
   MicroscopeProperties.cpp
   SegParams.cpp
   )
   
TARGET_LINK_LIBRARIES (core ${PCL_LIBRARIES})
