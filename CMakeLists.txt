
list(APPEND EXTRA_ZEPHYR_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
)    

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(TimelapseSensor)

target_sources(app PRIVATE 
    src/main.c
    src/GenericCentralApp.c
    src/scs_client.c
    src/iir_filter.c)