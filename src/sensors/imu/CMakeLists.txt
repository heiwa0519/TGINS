cmake_minimum_required(VERSION 3.0.0)

project(${FImuLib})

file(GLOB_RECURSE header_files ${libFImuSrc}/*.h)
file(GLOB_RECURSE source_files ${libFImuSrc}/*.cc)

include_directories(${libFImuSrc}
                    ${libCommonSrc}
                    ${libFGnssSrc}
                    ${libFusingSrc})

add_library(${PROJECT_NAME} ${header_files} ${source_files})
target_link_libraries(${PROJECT_NAME}  ${CommonLib})

if (CMAKE_SYSTEM_NAME MATCHES "Windows")
    link_directories(${ROOT}/build/Lib/Debug)
    link_directories(${ROOT}/build/Lib/Release)
    link_directories(${ROOT}/build/Lib/RelWithDebInfo)
    link_directories(${ROOT}/build/Lib/MinSizeRel)
else ()
    link_directories(${ROOT}/build/Lib)
endif ()
