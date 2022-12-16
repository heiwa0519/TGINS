cmake_minimum_required(VERSION 3.0.0)

project(${MiscLib})

file(GLOB_RECURSE header_files ${libMiscSrc}/*.h)
file(GLOB_RECURSE source_files ${libMiscSrc}/*.cc)

include_directories(${libMiscSrc}
                    ${libCommonSrc}
                    ${libFusingSrc}
                    ${libFImuSrc}
                    ${libFGnssSrc}
                    ${libFusingSrc})

add_library(${PROJECT_NAME} ${header_files} ${source_files})

target_link_libraries(${PROJECT_NAME} ${CommonLib}
                                      ${FImuLib}
                                      ${FGnssLib}
                                      ${FusingLib})

if (CMAKE_SYSTEM_NAME MATCHES "Windows")
    link_directories(${ROOT}/build/Lib/Debug)
    link_directories(${ROOT}/build/Lib/Release)
    link_directories(${ROOT}/build/Lib/RelWithDebInfo)
    link_directories(${ROOT}/build/Lib/MinSizeRel)
else ()
    link_directories(${ROOT}/build/Lib)
endif ()
