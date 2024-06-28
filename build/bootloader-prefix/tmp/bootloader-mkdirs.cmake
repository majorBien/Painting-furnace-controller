# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/opt/esp-idf/components/bootloader/subproject"
  "/home/majorbien/eclipse-workspace/extreme_design_custom/build/bootloader"
  "/home/majorbien/eclipse-workspace/extreme_design_custom/build/bootloader-prefix"
  "/home/majorbien/eclipse-workspace/extreme_design_custom/build/bootloader-prefix/tmp"
  "/home/majorbien/eclipse-workspace/extreme_design_custom/build/bootloader-prefix/src/bootloader-stamp"
  "/home/majorbien/eclipse-workspace/extreme_design_custom/build/bootloader-prefix/src"
  "/home/majorbien/eclipse-workspace/extreme_design_custom/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/majorbien/eclipse-workspace/extreme_design_custom/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/majorbien/eclipse-workspace/extreme_design_custom/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
