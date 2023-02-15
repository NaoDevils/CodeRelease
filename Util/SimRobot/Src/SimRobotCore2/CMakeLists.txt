cmake_minimum_required(VERSION 3.20)
project(SimRobotCore2)

find_package(Qt6 6.3 COMPONENTS Core Gui OpenGL OpenGLWidgets Widgets REQUIRED)
find_package(ODE 0.16 REQUIRED)
find_package(GLEW 2.2 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 3.4 REQUIRED)

add_library(${PROJECT_NAME}
    MODULE
        ActuatorsWidget.cpp
        ActuatorsWidget.h
        CoreModule.cpp
        CoreModule.h
        Parser/Element.cpp
        Parser/Element.h
        Parser/Parser.cpp
        Parser/Parser.h
        Parser/Reader.cpp
        Parser/Reader.h
        Platform/Assert.cpp
        Platform/Assert.h
        Platform/OffscreenRenderer.cpp
        Platform/OffscreenRenderer.h
        Platform/OpenGL.h
        Platform/System.cpp
        Platform/System.h
        SensorWidget.cpp
        SensorWidget.h
        SimObjectRenderer.cpp
        SimObjectRenderer.h
        SimObjectWidget.cpp
        SimObjectWidget.h
        SimRobotCore2.h
        SimRobotCore2.qrc
        Simulation/Actuators/Actuator.cpp
        Simulation/Actuators/Actuator.h
        Simulation/Actuators/Hinge.cpp
        Simulation/Actuators/Hinge.h
        Simulation/Actuators/Joint.cpp
        Simulation/Actuators/Joint.h
        Simulation/Actuators/Slider.cpp
        Simulation/Actuators/Slider.h
        Simulation/Appearances/Appearance.cpp
        Simulation/Appearances/Appearance.h
        Simulation/Appearances/BoxAppearance.cpp
        Simulation/Appearances/BoxAppearance.h
        Simulation/Appearances/CapsuleAppearance.cpp
        Simulation/Appearances/CapsuleAppearance.h
        Simulation/Appearances/ComplexAppearance.cpp
        Simulation/Appearances/ComplexAppearance.h
        Simulation/Appearances/CylinderAppearance.cpp
        Simulation/Appearances/CylinderAppearance.h
        Simulation/Appearances/SphereAppearance.cpp
        Simulation/Appearances/SphereAppearance.h
        Simulation/Axis.cpp
        Simulation/Axis.h
        Simulation/Body.cpp
        Simulation/Body.h
        Simulation/Compound.cpp
        Simulation/Compound.h
        Simulation/Geometries/BoxGeometry.cpp
        Simulation/Geometries/BoxGeometry.h
        Simulation/Geometries/CapsuleGeometry.cpp
        Simulation/Geometries/CapsuleGeometry.h
        Simulation/Geometries/CylinderGeometry.cpp
        Simulation/Geometries/CylinderGeometry.h
        Simulation/Geometries/Geometry.cpp
        Simulation/Geometries/Geometry.h
        Simulation/Geometries/SphereGeometry.cpp
        Simulation/Geometries/SphereGeometry.h
        Simulation/GraphicalObject.cpp
        Simulation/GraphicalObject.h
        Simulation/Masses/BoxMass.cpp
        Simulation/Masses/BoxMass.h
        Simulation/Masses/InertiaMatrixMass.cpp
        Simulation/Masses/InertiaMatrixMass.h
        Simulation/Masses/Mass.cpp
        Simulation/Masses/Mass.h
        Simulation/Masses/SphereMass.cpp
        Simulation/Masses/SphereMass.h
        Simulation/Motors/Motor.h
        Simulation/Motors/PT2Motor.cpp
        Simulation/Motors/PT2Motor.h
        Simulation/Motors/ServoMotor.cpp
        Simulation/Motors/ServoMotor.h
        Simulation/Motors/VelocityMotor.cpp
        Simulation/Motors/VelocityMotor.h
        Simulation/PhysicalObject.cpp
        Simulation/PhysicalObject.h
        Simulation/Scene.cpp
        Simulation/Scene.h
        Simulation/Sensors/Accelerometer.cpp
        Simulation/Sensors/Accelerometer.h
        Simulation/Sensors/ApproxDistanceSensor.cpp
        Simulation/Sensors/ApproxDistanceSensor.h
        Simulation/Sensors/Camera.cpp
        Simulation/Sensors/Camera.h
        Simulation/Sensors/CollisionSensor.cpp
        Simulation/Sensors/CollisionSensor.h
        Simulation/Sensors/DepthImageSensor.cpp
        Simulation/Sensors/DepthImageSensor.h
        Simulation/Sensors/Gyroscope.cpp
        Simulation/Sensors/Gyroscope.h
        Simulation/Sensors/ObjectSegmentedImageSensor.cpp
        Simulation/Sensors/ObjectSegmentedImageSensor.h
        Simulation/Sensors/Sensor.cpp
        Simulation/Sensors/Sensor.h
        Simulation/Sensors/SingleDistanceSensor.cpp
        Simulation/Sensors/SingleDistanceSensor.h
        Simulation/SimObject.cpp
        Simulation/SimObject.h
        Simulation/Simulation.cpp
        Simulation/Simulation.h
        Simulation/UserInput.cpp
        Simulation/UserInput.h
        Tools/Math.h
        Tools/Math/Approx.h
        Tools/Math/Constants.h
        Tools/Math/Eigen.h
        Tools/Math/EigenMatrixBaseExtensions.h
        Tools/Math/Pose3f.h
        Tools/Math/Rotation.h
        Tools/Math/RotationMatrix.cpp
        Tools/Math/RotationMatrix.h
        Tools/Matrix3x3.cpp
        Tools/Matrix3x3.h
        Tools/ODETools.h
        Tools/OpenGLTools.cpp
        Tools/OpenGLTools.h
        Tools/Pose3.h
        Tools/Texture.cpp
        Tools/Texture.h
        Tools/Vector3.cpp
        Tools/Vector3.h
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::OpenGL
        Qt6::OpenGLWidgets
        Qt6::Widgets
        GLEW::GLEW
        ODE::ODE
        OpenGL::GLU
        Threads::Threads
        Eigen3::Eigen
        SimRobotAPI
)

target_optimize(${PROJECT_NAME} FULL RelWithDebInfo Release)

set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOMOC ON)
set_property(TARGET ${PROJECT_NAME} PROPERTY AUTORCC ON)

add_library("${PROJECT_NAME}API" INTERFACE)
target_include_directories("${PROJECT_NAME}API" INTERFACE .)
