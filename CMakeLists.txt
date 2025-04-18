# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "1.6.0")

# Add this line to disable the specific warning
set(compile_options ${compile_options} 
  "-Wno-error=format" 
  "-Wno-error=maybe-uninitialized"
  "-Wno-missing-field-initializers"
  "-Wno-error=stringop-overflow"
  "-DPSRAM_SIZE_KB=8192"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(xiaozhi)

