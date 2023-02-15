cmake_minimum_required(VERSION 3.20)
project(SimRobotEditor)

find_package(Qt6 6.3 COMPONENTS Core Gui Widgets REQUIRED)

add_library(${PROJECT_NAME}
    MODULE
        SimRobotEditor.qrc
        EditorModule.cpp
        EditorWidget.cpp
        SyntaxHighlighter.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        SimRobotAPI
)

target_optimize(${PROJECT_NAME} FULL RelWithDebInfo Release)

set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOMOC ON)
set_property(TARGET ${PROJECT_NAME} PROPERTY AUTORCC ON)

add_library("${PROJECT_NAME}API" INTERFACE)
target_include_directories("${PROJECT_NAME}API" INTERFACE .)
