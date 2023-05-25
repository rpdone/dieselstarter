# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/r.perez/esp4/esp-idf/components/bootloader/subproject"
  "C:/Users/r.perez/Documents/ESP/blink/build/bootloader"
  "C:/Users/r.perez/Documents/ESP/blink/build/bootloader-prefix"
  "C:/Users/r.perez/Documents/ESP/blink/build/bootloader-prefix/tmp"
  "C:/Users/r.perez/Documents/ESP/blink/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/r.perez/Documents/ESP/blink/build/bootloader-prefix/src"
  "C:/Users/r.perez/Documents/ESP/blink/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/r.perez/Documents/ESP/blink/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
