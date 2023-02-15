# Set paths for Conan packages
list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

# Specify find root path for cross compilation
# https://github.com/conan-io/conan/issues/4967#issuecomment-891754084
list(APPEND CMAKE_FIND_ROOT_PATH "${CMAKE_BINARY_DIR}")
list(APPEND CMAKE_FIND_ROOT_PATH "${CMAKE_SOURCE_DIR}")

# Hides annoying "library found" messages in CMake output
set(CONAN_CMAKE_SILENT_OUTPUT ON)

# Prefer Conan packages over system packages
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)

if(CONAN_INSTALL)
    if(BUILD_ROBOT)
        list(APPEND
            CONAN_IMPORTS
                "lib, *.so* -> ./lib"
        )
        set(CMAKE_BUILD_RPATH "$ORIGIN/lib")
    else()
        list(APPEND
            CONAN_REQUIRES
                qt/6.3.1
                glew/2.2.0
                ode/0.16.2
        )
        list(APPEND
            CONAN_OPTIONS
                qt:qt5compat=True
                qt:qtsvg=True
                qt:shared=True
                qt:with_pq=False
        )
        list(APPEND
            CONAN_GENERATORS
                qt
        )
    endif()

    get_property(multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)

    if(multi_config)
        set(CONAN_BUILD_TYPES ${CMAKE_CONFIGURATION_TYPES})
    else()
        set(CONAN_BUILD_TYPES ${CMAKE_BUILD_TYPE})
    endif()

    conan_cmake_configure(
        REQUIRES
            snappy/1.1.9
            nlohmann_json/3.11.2
            libjpeg-turbo/2.1.4
            taskflow/3.5.0
            protobuf/3.21.4
            eigen/3.4.0
            kissfft/131.1.0
            tensorflow-lite/2.10.0
            portaudio/19.7.0
            flite/2.2.1
            ${CONAN_REQUIRES}
        OPTIONS
            ${CONAN_OPTIONS}
        GENERATORS
            cmake
            cmake_find_package_multi
            ${CONAN_GENERATORS}
        IMPORTS
            ${CONAN_IMPORTS}
    )

    # Copy shared libraries on Windows.
    # Normally, conan imports do the same job, but all files are replaced each time.
    # When an application is running during configuration, loaded .dll files are read-only, which causes permission errors.
    # In contrast, CMake copies files on modification only.
    function(conan_copy_shared_libraries dest)
        conan_load_buildinfo()
        set(CONAN_DLLS ${CONAN_BIN_DIRS})
        list(TRANSFORM CONAN_DLLS APPEND "/*.dll")
        file(GLOB CONAN_DLLS ${CONAN_DLLS})
        file(COPY ${CONAN_DLLS} DESTINATION ${dest})
    endfunction()

    foreach(type ${CONAN_BUILD_TYPES})
        conan_cmake_autodetect(settings BUILD_TYPE ${type})
    
        # Use Release dependencies for RelWithDebInfo builds
        if(type STREQUAL "RelWithDebInfo")
            list(APPEND settings *:build_type=Release)
        endif()

        conan_cmake_install(
            PATH_OR_REFERENCE
                .
            BUILD
                outdated
                cascade
            REMOTE
                naodevils
            SETTINGS
                ${settings}
            UPDATE
        )
    
        if(multi_config)
            conan_copy_shared_libraries("${CMAKE_BINARY_DIR}/${type}")
            if(NOT BUILD_ROBOT)
                file(COPY "${CMAKE_BINARY_DIR}/qt.conf" DESTINATION "${CMAKE_BINARY_DIR}/${type}")
            endif()
        else()
            conan_copy_shared_libraries("${CMAKE_BINARY_DIR}")
        endif()
    endforeach()

    # Fix WSL1 bug
    # https://github.com/microsoft/WSL/issues/3023
    if(CMAKE_SYSTEM MATCHES "^Linux-.+-Microsoft$")
        execute_process(COMMAND bash -c "strip --remove-section=.note.ABI-tag ~/.conan/data/qt/*/_/_/package/*/lib/libQt6Core.so.6.*")
    endif()

    # Save timestamp of last "conan install" execution
    file(TOUCH "${CONAN_TOUCH_FILE}")
endif()
