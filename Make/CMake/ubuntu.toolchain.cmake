if(WIN32)
    message(FATAL_ERROR "Unsupported operating system!")
endif()

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=silvermont -mtune=silvermont")

get_filename_component(ROOT_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)

# cache variable for VS to be able to detect sysroot in CMakeCache.txt
set(CMAKE_SYSROOT "${ROOT_DIR}" CACHE FILEPATH "System root for cross compilation")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
