cmake_minimum_required(VERSION 2.6)

project(Rtk_gtsam)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_INSTALL_PREFIX output)
SET(CMAKE_C_COMPILER gcc)
SET(CMAKE_CXX_COMPILER g++)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -fpic -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fpic -std=c++11")


##Find boost
find_package(Boost REQUIRED COMPONENTS
filesystem  
)

if(NOT Boost_FOUND)
    message("Not found Boost")
endif()

include_directories(${Boost_INCLUDE_DIRS})
message("${Boost_INCLUDE_DIRS}")
message("${Boost_LIBRARIES}")
include_directories(./rtk)
aux_source_directory(./rtk SRC_LIST_rtk)
##Find gtsam
# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)
#include(GtsamBuildTypes) # Load build type flags and default to Debug mode
#include(GtsamTesting)    # Easy functions for creating unit tests and scripts
#include(GtsamMatlabWrap) # Automatic MATLAB wrapper generation

# Ensure that local folder is searched before library folders
#include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

###################################################################################
# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

###################################################################################
# Build static library from common sources
#set(CONVENIENCE_LIB_NAME ${PROJECT_NAME})
#add_library(${CONVENIENCE_LIB_NAME} STATIC include/car/car_lib.h src/car_lib.cpp)
#target_link_libraries(${CONVENIENCE_LIB_NAME} gtsam)


##include head file
include_directories(
${PROJECT_SOURCE_DIR})

add_executable(Rtk_gtsam ${PROJECT_SOURCE_DIR}/main.cpp ${PROJECT_SOURCE_DIR}/global.cpp ${PROJECT_SOURCE_DIR}/RTK_Optimizer.cpp ${SRC_LIST_rtk} ${PROJECT_SOURCE_DIR}/Rtkpost.cpp)

target_link_libraries(Rtk_gtsam ${Boost_LIBRARIES} gtsam)

