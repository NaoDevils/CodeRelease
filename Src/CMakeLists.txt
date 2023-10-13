cmake_minimum_required(VERSION 3.20)

project(Nao)

find_package(Threads REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(libjpeg-turbo 2.1 REQUIRED)
find_package(nlohmann_json 3.11 REQUIRED)
find_package(Taskflow 3.6 REQUIRED)
find_package(Snappy 1.1 REQUIRED)
find_package(protobuf 3.21 REQUIRED)
find_package(kissfft 131.1 REQUIRED)
find_package(tensorflowlite 2.10 REQUIRED)
find_package(portaudio 19.7 REQUIRED)
find_package(flite 2.2 REQUIRED)

if(BUILD_ROBOT)
    find_library(LIBRT rt REQUIRED)

    add_executable(${PROJECT_NAME} "")
    set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "naodevils")
else()
    add_subdirectory(Controller)
    
    add_library(${PROJECT_NAME} MODULE "")
    set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "SimulatedNao")
endif()

# protobuf generation does not work in subdirectory
protobuf_generate_cpp(
    PROTO_SRCS
    PROTO_HDRS 
    Tools/Protobuf/imageLabelData.proto
)
target_sources(${PROJECT_NAME} PRIVATE ${PROTO_SRCS} ${PROTO_HDRS})

# hide warnings in protobuf generated file
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES COMPILE_FLAGS "-Wno-shorten-64-to-32")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES COMPILE_FLAGS "/wd4244 /wd4267")
endif()


# Download flite voices
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../Config/Voices/cmu_us_bdl.flitevox")
   message(STATUS "Downloading flite male voice from http://festvox.org/flite/packed/flite-2.1/voices")
   file(DOWNLOAD "http://festvox.org/flite/packed/flite-2.1/voices/cmu_us_bdl.flitevox" "${CMAKE_CURRENT_SOURCE_DIR}/../Config/Voices/cmu_us_bdl.flitevox")
endif()
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../Config/Voices/cmu_us_clb.flitevox")
   message(STATUS "Downloading flite female voice from http://festvox.org/flite/packed/flite-2.1/voices")
   file(DOWNLOAD "http://festvox.org/flite/packed/flite-2.1/voices/cmu_us_clb.flitevox" "${CMAKE_CURRENT_SOURCE_DIR}/../Config/Voices/cmu_us_clb.flitevox")
endif()

add_subdirectory(Modules)
add_subdirectory(Platform)
add_subdirectory(Processes)
add_subdirectory(Representations)
add_subdirectory(Tools)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Threads::Threads
        Eigen3::Eigen
        GameController
        libjpeg-turbo::libjpeg-turbo
        nlohmann_json::nlohmann_json
        Taskflow::Taskflow
        Snappy::Snappy
        protobuf::protobuf
        kissfft::kissfft
        tensorflowlite::tensorflowlite
        portaudio::portaudio
        flite::flite
)

if(BUILD_ROBOT)
    target_link_libraries(${PROJECT_NAME}
        PRIVATE
            ${LIBRT}
    )
    
    target_compile_definitions(${PROJECT_NAME}
        PRIVATE
            TARGET_ROBOT
    )

    target_optimize(${PROJECT_NAME} FULL RelWithDebInfo Release)
elseif(APPLE)
    enable_language(OBJC)
    enable_language(OBJCXX)

    target_link_libraries(${PROJECT_NAME} PRIVATE Controller "-framework IOKit")
    target_compile_definitions(${PROJECT_NAME} PRIVATE TARGET_SIM)

    target_optimize(${PROJECT_NAME} FULL Release MODERATE RelWithDebInfo)
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE Controller)
    target_compile_definitions(${PROJECT_NAME} PRIVATE TARGET_SIM)

    target_optimize(${PROJECT_NAME} FULL Release MODERATE RelWithDebInfo)
endif()

if(WIN32)
    target_link_libraries(${PROJECT_NAME}
        PRIVATE
            ws2_32
            winmm
    )
    
    target_compile_options(${PROJECT_NAME}
        PRIVATE
            /bigobj
    )
endif()

target_precompile_headers(${PROJECT_NAME}
    PRIVATE
        "Tools/Math/Eigen.h"
        
        "Tools/Boundary.h"
        "Tools/RingBufferWithSum.h"
        "Tools/Debugging/DebugDrawings.h"
        "Tools/Math/Geometry.h"
        "Tools/Math/Pose3f.h"
        "Tools/Math/Transformation.h"
        "Tools/MessageQueue/MessageQueue.h"
        "Tools/Module/Module.h"
        
        <algorithm>
        <list>
        <sstream>
)
