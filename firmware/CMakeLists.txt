cmake_minimum_required(VERSION 3.16)

# OpenQuarkyRover — M1 test firmware for the STEMpedia Quarky Intellio.
# ESP-IDF v5.x required. Build with `idf.py -B build set-target esp32s3 && idf.py build`.

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# esp-dl (transitive dep of esp-sr) references MALLOC_CAP_SIMD, which
# is only defined in ESP-IDF ≥ 5.4. On IDF 5.3 (what we ship on) the
# only esp-dl code that uses it is the vision path we don't enable, so
# aliasing it to 0 makes the `|`-mask a no-op and lets the link succeed
# without pulling in a whole IDF upgrade. Must run BEFORE project() so
# the property propagates to every managed-component target.
# Remove once we move to 5.4+.
idf_build_set_property(COMPILE_OPTIONS "-DMALLOC_CAP_SIMD=0" APPEND)

project(openquarkyrover_m1_test)
