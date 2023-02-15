cmake_minimum_required(VERSION 3.20)
project(libqxt)

find_package(Qt6 6.3 COMPONENTS Core Gui Widgets REQUIRED)

add_library(${PROJECT_NAME}
    STATIC
    qxtglobal.h
    qxtnamespace.h
    qxtspanslider.cpp
    qxtspanslider.h
    qxtspanslider_p.h
)

target_include_directories(${PROJECT_NAME} INTERFACE .)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
)

target_compile_definitions(${PROJECT_NAME} PRIVATE QXT_STATIC)

target_optimize(${PROJECT_NAME} FULL RelWithDebInfo Release)

set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOMOC ON)
set_property(TARGET ${PROJECT_NAME} PROPERTY AUTORCC ON)
set_property(SOURCE qxtnamespace.h PROPERTY SKIP_AUTOMOC ON)
