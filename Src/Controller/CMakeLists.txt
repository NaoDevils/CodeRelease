cmake_minimum_required(VERSION 3.20)
project(Controller)

find_package(Qt6 6.3 COMPONENTS Core Gui OpenGL OpenGLWidgets Svg SvgWidgets Widgets REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(GLEW 2.2.0 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(libjpeg-turbo 2.1 REQUIRED)
find_package(nlohmann_json 3.11 REQUIRED)
find_package(Snappy 1.1 REQUIRED)
find_package(portaudio 19.7 REQUIRED)
find_package(sol2 3.3.0 REQUIRED)

add_subdirectory(../../Util/qtpropertybrowser qtpropertybrowser)
add_subdirectory(../../Util/libqxt libqxt)

add_library(${PROJECT_NAME}
    STATIC
    AudioPlayer.cpp
    AudioPlayer.h
    ButtonToolBar.cpp
    ButtonToolBar.h
    ConsoleRoboCupCtrl.cpp
    ConsoleRoboCupCtrl.h
    Controller.qrc
    GameController.cpp
    GameController.h
    ImageViewAdapter.cpp
    ImageViewAdapter.h
    LocalRobot.cpp
    LocalRobot.h
    LogPlayer.cpp
    LogPlayer.h
    Platform/Joystick.h
    RemoteRobot.cpp
    RemoteRobot.h
    RoboCupCtrl.cpp
    RoboCupCtrl.h
    RobotConsole.cpp
    RobotConsole.h
    SimulatedRobot.cpp
    SimulatedRobot.h
    StdInConsole.cpp
    StdInConsole.h
    Platform/${PLATFORM}/Joystick.cpp
    Platform/${PLATFORM}/Joystick.h
    Representations/AnnotationInfo.cpp
    Representations/AnnotationInfo.h
    Representations/ModuleInfo.cpp
    Representations/ModuleInfo.h
    Representations/TimeInfo.cpp
    Representations/TimeInfo.h
    Views/AnnotationView/AnnotationView.cpp
    Views/AnnotationView/AnnotationView.h
    Views/AnnotationView/AnnotationWidget.cpp
    Views/AnnotationView/AnnotationWidget.h
    Views/CABSLBehaviorView.cpp
    Views/CABSLBehaviorView.h
    Views/CABSLGraphView.cpp
    Views/CABSLGraphView.h
    Views/ColorSpaceView.cpp
    Views/ColorSpaceView.h
    Views/ConsoleView.cpp
    Views/ConsoleView.h
    Views/DataView/DataView.cpp
    Views/DataView/DataView.h
    Views/DataView/DataWidget.cpp
    Views/DataView/DataWidget.h
    Views/DataView/EditorEventFilter.cpp
    Views/DataView/EditorEventFilter.h
    Views/DataView/PropertyEditorFactory.cpp
    Views/DataView/PropertyEditorFactory.h
    Views/DataView/PropertyManager.cpp
    Views/DataView/PropertyManager.h
    Views/DataView/PropertyTreeCreator.cpp
    Views/DataView/PropertyTreeCreator.h
    Views/DataView/PropertyTreeWriter.cpp
    Views/DataView/PropertyTreeWriter.h
    Views/DataView/TypeDeclarations.h
    Views/DataView/TypeDescriptor.cpp
    Views/DataView/TypeDescriptor.h
    Views/DotView.cpp
    Views/DotView.h
    Views/FieldView.cpp
    Views/FieldView.h
    Views/ImageView.cpp
    Views/ImageView.h
    Views/JointView.cpp
    Views/JointView.h
    Views/KickView/KickMenuBar.cpp
    Views/KickView/KickMenuBar.h
    Views/KickView/KickView.cpp
    Views/KickView/KickView.h
    Views/KickView/KickViewGLWidget.cpp
    Views/KickView/KickViewGLWidget.h
    Views/KickView/KickViewMath.cpp
    Views/KickView/KickViewMath.h
    Views/KickView/KickViewWidget.cpp
    Views/KickView/KickViewWidget.h
    Views/KickView/TabWidget.cpp
    Views/KickView/TabWidget.h
    Views/ModuleGraphView.cpp
    Views/ModuleGraphView.h
    Views/PlotView.cpp
    Views/PlotView.h
    Views/SensorView.cpp
    Views/SensorView.h
    Views/TimeView/TimeView.cpp
    Views/TimeView/TimeView.h
    Views/TimeView/TimeWidget.cpp
    Views/TimeView/TimeWidget.h
    Views/View3D.cpp
    Views/View3D.h
    Visualization/DebugDrawing.cpp
    Visualization/DebugDrawing.h
    Visualization/DebugDrawing3D.cpp
    Visualization/DebugDrawing3D.h
    Visualization/HeaderedWidget.cpp
    Visualization/HeaderedWidget.h
    Visualization/OpenGLMethods.cpp
    Visualization/OpenGLMethods.h
    Visualization/PaintMethods.cpp
    Visualization/PaintMethods.h
)

target_precompile_headers(${PROJECT_NAME}
    PRIVATE
        "../Tools/Math/Angle.h"
        "../Tools/Math/Eigen.h"
        "../Tools/Math/Pose3f.h"
        "../Tools/Module/Module.h"
        "../Tools/Debugging/DebugDrawings.h"

        "RobotConsole.h"
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ..
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        # SimulatedNao includes ConsoleRobCupCtrl => RobotConsole => DataView => qtpropertybrowser
        qtpropertybrowser
        SimRobotAPI
        SimRobotCore2API
        Qt6::Core
        Qt6::Gui
        Qt6::OpenGL
        Qt6::Widgets
    PRIVATE
        Qt6::OpenGLWidgets
        Qt6::Svg
        Qt6::SvgWidgets
        GLEW::GLEW
        OpenGL::GLU
        Threads::Threads
        Eigen3::Eigen
        libjpeg-turbo::libjpeg-turbo
        nlohmann_json::nlohmann_json
        libqxt
        SimRobotEditorAPI
        GameController
        Snappy::Snappy
        portaudio::portaudio
        sol2::sol2
)

target_compile_definitions(${PROJECT_NAME} PRIVATE TARGET_SIM)

target_optimize(${PROJECT_NAME} FULL RelWithDebInfo Release)

set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOMOC ON)
set_property(TARGET ${PROJECT_NAME} PROPERTY AUTORCC ON)
