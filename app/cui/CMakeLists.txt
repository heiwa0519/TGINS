cmake_minimum_required(VERSION 3.0.0)
cmake_policy(SET CMP0074 NEW)

project(FUSING-APP)

include_directories(${libCommonSrc}
                    ${libFImuSrc}
                    ${libFGnssSrc}
                    ${libFusingSrc}
                    ${libMiscSrc})


add_executable(TGINS TGINS.cc)
target_link_libraries(TGINS ${CommonLib} ${FGnssLib} ${FImuLib} ${FusingLib} ${MiscLib})

add_executable(compareSol compareSol.cc)
target_link_libraries(compareSol ${CommonLib} ${FGnssLib} ${FImuLib} ${MiscLib})

