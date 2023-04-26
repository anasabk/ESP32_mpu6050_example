cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(EXTRA_COMPONENT_DIRS 
        components $ENV{IDF_PATH}/examples/common_components/protocol_examples_common
                   ../components
        )
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32_mpu6050_example)