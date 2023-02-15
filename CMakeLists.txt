cmake_minimum_required(VERSION 3.20)

set(BUILD_ROBOT false CACHE BOOL "Compile code for the Nao robot")

include(Make/CMake/conan_init.cmake)

project(NaoDevils)

include(Make/CMake/settings.cmake)
include(Make/CMake/conan.cmake)
include(Make/CMake/warnings.cmake)

add_subdirectory(Src)
add_subdirectory(Util/GameController)

target_compile_options(Nao PRIVATE ${COMPILER_MORE_WARNINGS})

if(BUILD_ROBOT)
    add_subdirectory(Src/naodevilsbase)
    add_subdirectory(Src/sensorReader)
    add_dependencies(Nao sensorReader naodevilsbase)
    
    target_compile_options(sensorReader PRIVATE ${COMPILER_MOST_WARNINGS})
    target_compile_options(naodevilsbase PRIVATE ${COMPILER_MOST_WARNINGS})
else()
    add_subdirectory(Util/SimRobot)
    add_subdirectory(Src/Utils/dorsh)
    add_dependencies(SimRobot Nao)
endif()
