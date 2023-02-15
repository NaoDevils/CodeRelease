# Load Conan CMake integration
include("${CMAKE_CURRENT_LIST_DIR}/Conan/conan.cmake")

# Check if "conan install" is necessary (i.e., Conan related files were modified) and sets CONAN_INSTALL variable
set(CONAN_TOUCH_FILE "${CMAKE_BINARY_DIR}/conan.touch")
set(CONAN_INSTALL false)
file(GLOB_RECURSE CONAN_FILES "${CMAKE_CURRENT_LIST_DIR}/*")
foreach(conan_file IN LISTS CONAN_FILES)
    if(NOT EXISTS "${CONAN_TOUCH_FILE}" OR "${conan_file}" IS_NEWER_THAN "${CONAN_TOUCH_FILE}")
        set(CONAN_INSTALL true)
    endif()
endforeach()

if(CONAN_INSTALL)
    conan_check(VERSION 1.52.0 REQUIRED)
    conan_config_install(ITEM ${CMAKE_CURRENT_LIST_DIR}/Conan/Config)
endif()

# This includes the Nao Ubuntu system root for cross-compilation
function(conan_set_toolchain)
    if(CONAN_INSTALL)
        # write conanfile.txt in subdirectory
        set(CMAKE_CURRENT_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/toolchain")
        conan_cmake_configure(
            BUILD_REQUIRES
                nao-ubuntu/2.0
            GENERATORS
                cmake
        )
    
        conan_cmake_install(PATH_OR_REFERENCE . REMOTE naodevils)
    endif()

    conan_load_buildinfo()
    set(CMAKE_TOOLCHAIN_FILE "${CONAN_NAO-UBUNTU_ROOT}/root-sdk/ubuntu.toolchain.cmake" PARENT_SCOPE)
endfunction()

if(BUILD_ROBOT)
    conan_set_toolchain()
endif()
