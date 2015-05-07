# - Try to find libICARUS
#
#  ICARUS_FOUND - system has libICARUS
#  ICARUS_INCLUDE_DIR - the libICARUS include directories
#  ICARUS_LIBRARY - link these to use libICARUS

FIND_PATH(
  ICARUS_INCLUDE_DIR
  NAMES icarus/icarus.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../TrackingWork/DTAM_WORKING/icarus
    /usr/include
    /usr/local/include
)

#MESSAGE("ICARUS_LIBRARY = ${ICARUS_INCLUDE_DIR}")

FIND_PATH(
  ICARUS_INCLUDE_DIR_BUILD
  NAMES icarus/icarus.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../icarus/release/
    ${CMAKE_SOURCE_DIR}/../icarus/build/
    ${CMAKE_SOURCE_DIR}/../icarus/release/
    ${CMAKE_SOURCE_DIR}/../icarus/build/
    /usr/include
    /usr/local/include
)

FIND_LIBRARY(
  ICARUS_LIBRARY
  NAMES icarus
  PATHS
    ${CMAKE_SOURCE_DIR}/../icarus/release/icarus
    ${CMAKE_SOURCE_DIR}/../icarus/build/icarus
    ${CMAKE_SOURCE_DIR}/../icarus/release/icarus
    ${CMAKE_SOURCE_DIR}/../icarus/build/icarus
    /usr/lib
    /usr/local/lib
)


IF(ICARUS_INCLUDE_DIR AND ICARUS_LIBRARY)
  SET(ICARUS_FOUND TRUE)
ENDIF()


IF(ICARUS_FOUND)
   IF(NOT ICARUS_FIND_QUIETLY)
      MESSAGE(STATUS "Found ICARUS: ${ICARUS_LIBRARY}")
   ENDIF()
ELSE()
   IF(ICARUS_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find ICARUS")
   ENDIF()
ENDIF()
