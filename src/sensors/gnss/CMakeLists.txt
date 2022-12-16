cmake_minimum_required(VERSION 3.0.0)

project(${FGnssLib})

file(GLOB_RECURSE header_files ${libFGnssSrc}/*.h ${libFGnssSrc}/rtklib/*.h)
file(GLOB_RECURSE source_files ${libFGnssSrc}/*.cc ${libFGnssSrc}/rtklib/*.cc ${libFGnssSrc}/rtklib/rcv/*.cc)

include_directories(${libFGnssSrc}
                    ${libFGnssSrc}/rtklib
                    ${libCommonSrc}
                    ${libFusingSrc}
                    ${libMiscSrc})

add_library(${PROJECT_NAME} ${header_files} ${source_files})

target_link_libraries(${PROJECT_NAME} ${CommonLib} ${FusingLib} ${MiscLib})

if (CMAKE_SYSTEM_NAME MATCHES "Windows")
    link_directories(${ROOT}/build/Lib/Debug)
    link_directories(${ROOT}/build/Lib/Release)
    link_directories(${ROOT}/build/Lib/RelWithDebInfo)
    link_directories(${ROOT}/build/Lib/MinSizeRel)
else ()
    link_directories(${ROOT}/build/Lib)
endif ()
