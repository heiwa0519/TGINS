cmake_minimum_required(VERSION 3.0.0)

project(${FusingLib})

file(GLOB_RECURSE header_files ${libFusingSrc}/*.h)
file(GLOB_RECURSE source_files ${libFusingSrc}/*.cc)

include_directories(${libFusingSrc}
                    ${libFImuSrc}
                    ${libFGnssSrc}
                    ${libMiscSrc})

add_library(${PROJECT_NAME} ${header_files} ${source_files})

target_link_libraries(${PROJECT_NAME} ${CommonLib}
                                      ${FImuLib}
                                      ${FGnssLib}
                                      ${MiscLib})

if (CMAKE_SYSTEM_NAME MATCHES "Windows")
    link_directories(${ROOT}/build/Lib/Debug)
    link_directories(${ROOT}/build/Lib/Release)
    link_directories(${ROOT}/build/Lib/RelWithDebInfo)
    link_directories(${ROOT}/build/Lib/MinSizeRel)
else ()
    link_directories(${ROOT}/build/Lib)
endif ()
