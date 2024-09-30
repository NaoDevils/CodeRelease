set(CMAKE_CONFIGURATION_TYPES RelWithDebInfo Debug Release CACHE STRING "" FORCE)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_BUILD_TYPE IN_LIST CMAKE_CONFIGURATION_TYPES)
    message(FATAL_ERROR "Invalid build type!")
endif()

# enable C++20 globally
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# disable compiler-specific language extensions
set(CMAKE_CXX_EXTENSIONS OFF)

# Set library output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
if(APPLE)
    get_property(multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
    if(multi_config)
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/$<CONFIG>/SimRobot.app/Contents/Resources")
    else()
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/SimRobot.app/Contents/Resources")
    endif()
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# enable ccache if present
if(ENABLE_CCACHE)
    find_program(CCACHE_PROGRAM ccache)
    if(CCACHE_PROGRAM)
        message(STATUS "Ccache found")
        set(CMAKE_C_COMPILER_LAUNCHER   "${CCACHE_PROGRAM}")
        set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    endif()
endif()

# general and optimization settings
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND NOT CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pipe")
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mssse3")
    endif()

    if(BUILD_ROBOT)
        # Use DWARFv4 as long as we are using Ubuntu 20.04 with gdb 9 on Nao
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -gdwarf-4")
    else()
        # optimize for this architecture
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
    endif()

    set(COMPILER_NO_OPTIMIZATION_FLAGS
        -g  # create debug information
    )
    set(COMPILER_MODERATE_OPTIMIZATION_FLAGS
        -finline-hint-functions
        -g  # create debug information
    )
    set(COMPILER_FULL_OPTIMIZATION_FLAGS
        -O3 # optimize for speed
        -g1 # create minimal debug information
    )
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" OR CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GF") # string pooling
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /permissive-") # strict C++ conformance
    if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:preprocessor") # enable preprocessor conformance mode
    elseif(CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native") # optimize for this architecture
    endif()

    if (CMAKE_GENERATOR MATCHES "Visual Studio")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP") # multiple processes
    endif()

    # always enable linking with debug symbols
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /DEBUG")
    set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /DEBUG")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /DEBUG")

    # Ccache needs debug symbols integrated into obj files
    if(CCACHE_PROGRAM)
        set(DEBUG_FLAG /Z7)
    else()
        set(DEBUG_FLAG /Zi)
    endif()
    
    set(COMPILER_NO_OPTIMIZATION_FLAGS
        /Od   # disable optimization
        /ZI   # create debugging information for edit-and-continue
        /RTC1 # run-time stack frame and variable initalization checking
    )
    set(COMPILER_MODERATE_OPTIMIZATION_FLAGS
        /Od   # disable optimization
        ${DEBUG_FLAG}  # create debugging information 
        /Ob1  # allows expansion of functions marked inline (e.g., speeds up Eigen significantly)
    )
    set(COMPILER_FULL_OPTIMIZATION_FLAGS
        /O2   # optimize for speed
        ${DEBUG_FLAG}   # create debugging information
    )
else()
    message(FATAL_ERROR "Unsupported compiler!")
endif()

# reset configuration-dependent compile flags
foreach(type IN LISTS CMAKE_CONFIGURATION_TYPES)
    string(TOUPPER ${type} type_upper)
    set("CMAKE_CXX_FLAGS_${type_upper}" "")
    set("CMAKE_EXE_LINKER_FLAGS_${type_upper}" "")
    set("CMAKE_MODULE_LINKER_FLAGS_${type_upper}" "")
    set("CMAKE_SHARED_LINKER_FLAGS_${type_upper}" "")
endforeach()

# turn on full or moderate compiler optimizations for given target depending on build config
function(target_optimize target)
    cmake_parse_arguments(PARSE_ARGV 1 PREFIX "" "" "FULL;MODERATE")

    target_compile_options(${target}
        PRIVATE
            $<IF:$<IN_LIST:$<CONFIG>,${PREFIX_FULL}>,${COMPILER_FULL_OPTIMIZATION_FLAGS},$<IF:$<IN_LIST:$<CONFIG>,${PREFIX_MODERATE}>,${COMPILER_MODERATE_OPTIMIZATION_FLAGS},${COMPILER_NO_OPTIMIZATION_FLAGS}>>
    )

    target_compile_definitions(${target}
        PRIVATE
            $<$<CONFIG:Debug>:_DEBUG>
            $<$<CONFIG:Release>:NDEBUG>
    )
endfunction()

# turn on full or moderate compiler optimizations for given files of given target depending on build config
function(files_optimize target)
    cmake_parse_arguments(PARSE_ARGV 1 PREFIX "" "" "FILES;FULL;MODERATE")
    set_source_files_properties(
        ${PREFIX_FILES}

        TARGET_DIRECTORY
            ${target}
        PROPERTIES
            COMPILE_OPTIONS "$<IF:$<IN_LIST:$<CONFIG>,${PREFIX_FULL}>,${COMPILER_FULL_OPTIMIZATION_FLAGS},$<IF:$<IN_LIST:$<CONFIG>,${PREFIX_MODERATE}>,${COMPILER_MODERATE_OPTIMIZATION_FLAGS},${COMPILER_NO_OPTIMIZATION_FLAGS}>>"
            SKIP_PRECOMPILE_HEADERS ON
    )
endfunction()

# enable pthreads
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

if(WIN32)
    set(PLATFORM Windows)
    
    add_compile_definitions(WINDOWS)
    
    # disable warnings for insecure C++ functions
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
    
    # disable MIN/MAX preprocessor macros
    add_compile_definitions(NOMINMAX)
elseif(APPLE)
    set(PLATFORM macOS)
    add_compile_definitions(MACOS)
else()
    set(PLATFORM Linux)
    
    add_compile_definitions(LINUX)
endif()

# replace RelWithDebInfo with Develop
add_compile_definitions(CONFIGURATION=$<IF:$<STREQUAL:$<CONFIG>,RelWithDebInfo>,Develop,$<CONFIG>>)
