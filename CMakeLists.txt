cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32-S3_ServoDriver)
spiffs_create_partition_image(storage spiffs_image FLASH_IN_PROJECT)
