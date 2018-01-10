ADD_LIBRARY(core SHARED  
   MicroscopeProperties.cpp
   )
   
TARGET_LINK_LIBRARIES (core ${PCL_LIBRARIES})
