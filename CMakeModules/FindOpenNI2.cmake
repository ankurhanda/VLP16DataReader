# - Try to find libfreenect
#

#MESSAGE(STATUS ${CMAKE_SOURCE_DIR}/../libfreenect/include)

FIND_PATH(
  OpenNI2_INCLUDE_DIR
  NAMES OpenNI.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../OpenNI2/Include
    /usr/include
    /usr/local/include
)

FIND_LIBRARY(
  OpenNI2_LIBRARY
  NAMES OpenNI2
  PATHS
    ${CMAKE_SOURCE_DIR}/../OpenNI2/Bin/x64-Release
    /usr/lib
    /usr/local/lib
)


IF(OpenNI2_INCLUDE_DIR AND OpenNI2_LIBRARY)
  SET(OpenNI2_FOUND TRUE)
ENDIF()


IF(OpenNI2_FOUND)
   IF(NOT OpenNI2_FIND_QUIETLY)
      MESSAGE(STATUS "Found OpenNI2: ${OpenNI2_LIBRARY}")
   ENDIF()
ELSE()
   IF(OpenNI2_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find OpenNI2")
   ENDIF()
ENDIF()

