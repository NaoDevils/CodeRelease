cmake_minimum_required(VERSION 3.20)
project(SimRobot)

find_package(Qt6 6.3 COMPONENTS Core Gui OpenGL OpenGLWidgets Svg Widgets REQUIRED)
find_package(Threads REQUIRED)

add_executable(${PROJECT_NAME}
        WIN32
        MACOSX_BUNDLE
        Icons/SimRobot.icns
        Main.cpp
        MainWindow.cpp
        MainWindow.h
        RegisteredDockWidget.cpp
        RegisteredDockWidget.h
        SceneGraphDockWidget.cpp
        SceneGraphDockWidget.h
        SimRobot.h
        SimRobot.qrc
        SimRobot.rc
        StatusBar.cpp
        StatusBar.h
        resource.h
)

# https://github.com/wjakob/instant-meshes/blob/011fa44ab72037cbc16535090a63ead41963c1e5/CMakeLists.txt#L151
set_target_properties(${PROJECT_NAME}
        PROPERTIES
        MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME}
        MACOSX_BUNDLE_BUNDLE_GUI_IDENTIFIER "edu.tudo.simrobot"
        MACOSX_BUNDLE_ICON_FILE SimRobot.icns
        MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/Icons/MacOSXBundleInfo.plist.in
)
set_source_files_properties(Icons/SimRobot.icns PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")

target_link_libraries(${PROJECT_NAME}
        PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::OpenGL
        Qt6::OpenGLWidgets
        Qt6::Svg
        Qt6::Widgets
        Threads::Threads
)

target_optimize(${PROJECT_NAME} FULL RelWithDebInfo Release)

set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOMOC ON)
set_property(TARGET ${PROJECT_NAME} PROPERTY AUTORCC ON)
qt6_disable_unicode_defines(${PROJECT_NAME})

add_library("${PROJECT_NAME}API" INTERFACE)
target_include_directories("${PROJECT_NAME}API" INTERFACE .)
