cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(TIS_sensor)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)
target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/ble_service.c)
target_sources(app PRIVATE src/adc_pwm.c)