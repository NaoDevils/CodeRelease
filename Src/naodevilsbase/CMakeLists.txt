cmake_minimum_required(VERSION 3.20)

project(naodevilsbase)

find_package(Threads REQUIRED)
find_package(nlohmann_json 3.11 REQUIRED)
find_library(LIBRT rt REQUIRED)

add_executable(${PROJECT_NAME}
    naodevilsbase.cpp
    naodevilsbase.h
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Threads::Threads
        nlohmann_json::nlohmann_json
        ${LIBRT}
)

target_include_directories(${PROJECT_NAME} PRIVATE ..)

target_optimize(${PROJECT_NAME} FULL RelWithDebInfo Release)
