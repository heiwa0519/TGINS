cmake_minimum_required(VERSION 3.0.0)
cmake_policy(SET CMP0074 NEW)

project(TGINS)
message("Welcome to FUSING...")

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    add_compile_options(-O3)
    add_compile_options(-pthread)
    add_compile_options(-w)
    add_compile_options(-m64)
    add_compile_options(-lz)
    add_compile_options(-lstdc++)
elseif(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    add_compile_options(-g)
    add_compile_options(-O3)
    add_compile_options(-Wall)
    add_compile_options(-pthread)
endif()

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pthread")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
set(CMAKE_CXX_FLAGS "-std=c++11 -O0 -fPIC")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "-msse3")
set(CMAKE_C_FLAGS "-msse3")
set(CMAKE_C_FLAGS "-mavx")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DELPP_NO_DEFAULT_LOG_FILE -DELPP_DISABLE_LOG_FILE_FROM_ARGs -DELPP_FRESH_LOG_FILE")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DELPP_DISABLE_LOGS ")
# DELPP_DISABLE_DEBUG_LOGS

#Set the ROOT and subdirectory, you should put the CMakeList.txt in these file directories
set(ROOT ${PROJECT_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${ROOT}/build/Bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${ROOT}/build/Lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ROOT}/build/Lib)

set(CMAKE_DEBUG_POSTFIX "d")
set(CMAKE_RELEASE_POSTFIX "")
set(CMAKE_RELWITHDEBINFO_POSTFIX "rd")
set(CMAKE_MINSIZEREL_POSTFIX "s")

#Choose different compilation configurations according to VS compilation
if(CMAKE_BUILD_TYPE MATCHES "Release")
    set(CMAKE_BUILD_POSTFIX "${CMAKE_RELEASE_POSTFIX}")
elseif(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_BUILD_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
elseif(CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
    set(CMAKE_BUILD_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}")
elseif(CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
    set(CMAKE_BUILD_POSTFIX "${CMAKE_MINSIZEREL_POSTFIX}")
else()
    set(CMAKE_BUILD_POSTFIX "")
endif()

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# yaml-cpp
find_package(yaml-cpp REQUIRED)

set(CommonLib       CommonLib)
set(FImuLib         FImuLib)
set(FGnssLib        FGnssLib)
set(FCameraLib      FCameraLib)
set(FUwbLib         FUwbLib)
set(FLidarLib       FLidarLib)
set(FRadarLib       FRadarLib)
set(FusingLib       FusingLib)
set(MiscLib          MiscLib)


set(libCommonSrc      ${PROJECT_SOURCE_DIR}/src/common)
set(libFImuSrc        ${PROJECT_SOURCE_DIR}/src/sensors/imu)
set(libFGnssSrc       ${PROJECT_SOURCE_DIR}/src/sensors/gnss)
set(libFusingSrc      ${PROJECT_SOURCE_DIR}/src/fusing)
set(libMiscSrc        ${PROJECT_SOURCE_DIR}/src/misc)

add_subdirectory(${libCommonSrc}      ${ROOT}/build/${CommonLib})
add_subdirectory(${libFImuSrc}        ${ROOT}/build/${FImuLib})
add_subdirectory(${libFGnssSrc}       ${ROOT}/build/${FGnssLib})
add_subdirectory(${libFusingSrc}      ${ROOT}/build/${FusingLib})
add_subdirectory(${libMiscSrc}        ${ROOT}/build/${MiscLib})


add_subdirectory(app/cui ${ROOT}/build/App)

# group
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
SET_PROPERTY(TARGET ${CommonLib}    PROPERTY FOLDER "LIB")
SET_PROPERTY(TARGET ${FImuLib}      PROPERTY FOLDER "LIB")
SET_PROPERTY(TARGET ${FGnssLib}     PROPERTY FOLDER "LIB")
SET_PROPERTY(TARGET  ${MiscLib}      PROPERTY FOLDER "LIB")

#Output Messages for debug the Cmake
message(STATUS "operation system is : ${CMAKE_SYSTEM}")
message(STATUS "current platform is : ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMake version is    : ${CMAKE_SYSTEM_VERSION}")
message(STATUS "C compiler is       : ${CMAKE_C_COMPILER}")
message(STATUS "C++ compiler is     : ${CMAKE_CXX_COMPILER}")
message(STATUS "The program main directory is : ${ROOT}")
