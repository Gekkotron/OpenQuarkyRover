cmake_minimum_required(VERSION 3.16)

# OpenQuarkyRover — M1 test firmware for the STEMpedia Quarky Intellio.
# ESP-IDF v5.x required. Build with `idf.py -B build set-target esp32s3 && idf.py build`.

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(openquarkyrover_m1_test)
