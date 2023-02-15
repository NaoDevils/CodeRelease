cmake_minimum_required(VERSION 3.20)
project(qtpropertybrowser)

find_package(Qt6 6.3 COMPONENTS Core Gui Widgets Core5Compat REQUIRED)

add_library(${PROJECT_NAME}
    STATIC
    qtbuttonpropertybrowser.cpp
    qtbuttonpropertybrowser.h
    qteditorfactory.cpp
    qteditorfactory.h
    qtgroupboxpropertybrowser.cpp
    qtgroupboxpropertybrowser.h
    qtpropertybrowser.cpp
    qtpropertybrowser.h
    qtpropertybrowser.qrc
    qtpropertybrowserutils.cpp
    qtpropertybrowserutils_p.h
    qtpropertymanager.cpp
    qtpropertymanager.h
    qttreepropertybrowser.cpp
    qttreepropertybrowser.h
    qtvariantproperty.cpp
    qtvariantproperty.h
)

target_include_directories(${PROJECT_NAME} INTERFACE .)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        Qt6::Core5Compat
)

if(WIN32)
    target_compile_definitions(${PROJECT_NAME} PRIVATE QT_QTPROPERTYBROWSER_EXPORT)
endif()

target_optimize(${PROJECT_NAME} FULL RelWithDebInfo Release)

set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOMOC ON)
set_property(TARGET ${PROJECT_NAME} PROPERTY AUTORCC ON)
