cmake_minimum_required(VERSION 3.0.0)

project(${CommonLib})

file(GLOB_RECURSE header_files ${libCommonSrc}/*.h ../../3rdparty/*.hpp ../../3rdparty/*.h ../../3rdparty/ghc/*.hpp)
file(GLOB_RECURSE source_files ${libCommonSrc}/*.cc ../../3rdparty/*.cpp)

add_library(${PROJECT_NAME} ${header_files} ${source_files})
target_include_directories(${PROJECT_NAME} PUBLIC ../../3rdparty
                                                  ../../3rdparty/ghc
                                                  ${libCommonSrc}
                                                  ${libFImuSrc}
                                                  ${libFGnssSrc}/rtklib )
target_link_libraries(${PROJECT_NAME} yaml-cpp)


if (CMAKE_SYSTEM_NAME MATCHES "Windows")
    link_directories(${ROOT}/build/Lib/Debug)
    link_directories(${ROOT}/build/Lib/Release)
    link_directories(${ROOT}/build/Lib/RelWithDebInfo)
    link_directories(${ROOT}/build/Lib/MinSizeRel)
else ()
    link_directories(${ROOT}/build/Lib)
endif ()
