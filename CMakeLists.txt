# -DCMAKE_BUILD_TYPE=Debug
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
project( VLPHDLReader )

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -std=c++11 -std==gnu++11")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -std=c++11 -std=gnu++11")


# Overide with cmake -DCMAKE_BUILD_TYPE=Debug {dir}
IF( NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE Release )
ENDIF()

# Platform configuration vars
INCLUDE(SetPlatformVars)

FIND_PACKAGE(CUDA REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(GLEW REQUIRED)
FIND_PACKAGE(Pangolin REQUIRED)
FIND_PACKAGE(OpenMP REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(CVD REQUIRED)
FIND_PACKAGE(Cg REQUIRED)
FIND_PACKAGE(PCL REQUIRED)
FIND_PACKAGE(OpenCV2 REQUIRED)
FIND_PACKAGE(ZLIB REQUIRED)

FIND_PATH(CUDA_CUT_INCLUDE_DIR
   cutil.h
   PATHS ${CUDA_SDK_SEARCH_PATH}
   PATH_SUFFIXES "common/inc"
   DOC "Location of cutil.h"
   NO_DEFAULT_PATH
)


INCLUDE(${PROJECT_SOURCE_DIR}/CMakeModules/Find_SSE.cmake)

SET(
  SOURCES

)


INCLUDE_DIRECTORIES(
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
)

INCLUDE_DIRECTORIES(
  ${ASSIMP_INCLUDE_DIR}
  ${CUDA_CUT_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${freenect_INCLUDE_DIR}
  ${freenectsync_INCLUDE_DIR}
  ${OpenNI2_INCLUDE_DIR}
  ${OpenCV2_INCLUDE_PATH}
  ${Pangolin_INCLUDE_DIR}
  ${CG_INCLUDE_PATH}
  ${MinimalImgUtilities_INCLUDE_DIR}
  ${ICARUS_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${G2O_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
  ${CVD_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  ${OSG_INCLUDE_DIRS}
  ${ZLIB_INCLUDE_DIR}
  /usr/include/ni
  /usr/lib/x86_64-linux-gnu/glib-2.0/include
  /usr/include/glib-2.0
  /usr/lib/glib-2.0/include
)

LINK_DIRECTORIES(
  ${OPENCV_LIBRARY_DIRS}
  ${CUDA_SDK_ROOT_DIR}/lib
  ${CUDA_SDK_ROOT_DIR}/common/lib
  /usr/local/lib
  /usr/local/lib64/osgPlugins-3.3.3
  /usr/local/lib64
)

#message("PCL LIBRARY = " ${PCL_LIBRARIES})
message("PCL INCLUDE LIBRARY = " ${PCL_INCLUDE_DIRS})
message("Cg INCLUDE DIRS = ${CG_INCLUDE_PATH}")
message("PCAP LIBRARY = ${PCAP_LIBRARY}")


LINK_LIBRARIES(
  ${ASSIMP_LIBRARY}
  ${LIBRARY_NAME}
  ${Pangolin_LIBRARY}
  ${freenect_LIBRARY}
  ${OpenNI2_LIBRARY}
  ${freenectsync_LIBRARY}
  ${CG_LIBRARY}
  ${CG_GL_LIBRARY}
  ${MinimalImgUtilities_LIBRARY}
  ${ICARUS_LIBRARY}
  ${Eigen3_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_CORE_LIBRARY}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${CSPARSE_LIBRARY}
  ${CVD_LIBRARY}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${OSG_LIBRARY}
  ${ZLIB_LIBRARIES}
  -lOpenNI
  -lGLEW
  -lglut
  -lGLU
  -lcutil_x86_64
  -lGL
  -lgts
  -lglib-2.0
  -lboost_system
  -lpcl_search
  -lboost_filesystem
  -lboost_thread
  -lblas
  -llapack
  -lpcap
   /usr/lib/x86_64-linux-gnu/libopencv_core.so
)

IF(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
ENDIF()

MESSAGE(STATUS "Cutil at: ${CUDA_CUT_INCLUDE_DIR}")
MESSAGE(STATUS "CUDA_SDK at: ${CUDA_SDK_ROOT_DIR}")
SET(CUDA_NVCC_FLAGS  "-arch=sm_30"  "--use_fast_math" "-O3" "-lineinfo")

## The compiler option "--use_fast_math" forces the compiler to convert
## function(x) into __function(x) that has higher throughput.

ADD_EXECUTABLE(hdl_reader_test
              ./src/HDLGrabber/custom_hdl_grabber.h
              ./src/HDLGrabber/custom_hdl_grabber.cpp
              ./src/main.cpp)

