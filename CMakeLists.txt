add_library(SparkFun_Micro_OLED_Arduino_Library
    ${CMAKE_CURRENT_SOURCE_DIR}/src/SFE_MicroOLED.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hardware.cpp
)

target_link_libraries(SparkFun_Micro_OLED_Arduino_Library
    hardware_i2c
    pico_stdlib
    TwoWire
)

target_include_directories(SparkFun_Micro_OLED_Arduino_Library PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/util
)