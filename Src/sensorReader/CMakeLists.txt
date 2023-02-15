cmake_minimum_required(VERSION 3.20)

project(sensorReader)

find_package(nlohmann_json 3.11 REQUIRED)
find_package(Threads REQUIRED)
find_library(LIBRT rt REQUIRED)

add_executable(${PROJECT_NAME}
    sensorReader.cpp
    sensorReader.h
    ../Tools/Network/UdpComm.cpp
    ../Tools/Network/UdpComm.h
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Threads::Threads
        nlohmann_json::nlohmann_json
        ${LIBRT}
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ..
        ../naodevilsbase
)

target_compile_definitions(${PROJECT_NAME} PRIVATE SENSOR_READER)

target_optimize(${PROJECT_NAME} FULL Develop Release)
