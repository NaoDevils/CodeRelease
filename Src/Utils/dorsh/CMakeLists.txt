cmake_minimum_required(VERSION 3.20)
project(Dorsh)

find_package(Qt6 6.3 COMPONENTS Core Gui Widgets Concurrent Network REQUIRED)
find_package(nlohmann_json 3.11 REQUIRED)
find_package(Threads REQUIRED)

add_executable(${PROJECT_NAME}
    WIN32
    MACOSX_BUNDLE
    ../../Platform/BHAssert.h
    ../../Platform/CameraV6.h
    ../../Platform/Common/File.cpp
    ../../Platform/Common/File.h
    ../../Platform/DebugHandler.h
    ../../Platform/File.h
    ../../Platform/Semaphore.h
    ../../Platform/SimRobotQt/SystemCall.cpp
    ../../Platform/SimRobotQt/SystemCall.h
    ../../Platform/SystemCall.h
    ../../Platform/Thread.h
    ../../Platform/${PLATFORM}/BHAssert.cpp
    ../../Platform/${PLATFORM}/BHAssert.h
    ../../Platform/${PLATFORM}/Semaphore.cpp
    ../../Platform/${PLATFORM}/Semaphore.h
    ../../Platform/${PLATFORM}/Thread.h
    ../../Tools/Build.h
    ../../Tools/Enum.h
    ../../Tools/Global.cpp
    ../../Tools/Global.h
    ../../Tools/Network/TcpComm.cpp
    ../../Tools/Network/TcpComm.h
    ../../Tools/Network/UdpComm.cpp
    ../../Tools/Network/UdpComm.h
    ../../Tools/Streams/AutoStreamable.h
    ../../Tools/Streams/Eigen.h
    ../../Tools/Streams/InOut.cpp
    ../../Tools/Streams/InOut.h
    ../../Tools/Streams/InStreams.cpp
    ../../Tools/Streams/InStreams.h
    ../../Tools/Streams/OutStreams.cpp
    ../../Tools/Streams/OutStreams.h
    ../../Tools/Streams/RobotParameters.cpp
    ../../Tools/Streams/RobotParameters.h
    ../../Tools/Streams/SimpleMap.cpp
    ../../Tools/Streams/SimpleMap.h
    ../../Tools/Streams/StreamHandler.cpp
    ../../Tools/Streams/StreamHandler.h
    ../../Tools/Streams/Streamable.cpp
    ../../Tools/Streams/Streamable.h
    Initializer.cpp
    Initializer.h
    Session.cpp
    Session.h
    agents/DataAgent.cpp
    agents/DataAgent.h
    bhwrapper/Framework.cpp
    bhwrapper/Framework.h
    cmdlib/AbstractConsole.cpp
    cmdlib/AbstractConsole.h
    cmdlib/Command.h
    cmdlib/CommandAdapter.cpp
    cmdlib/CommandAdapter.h
    cmdlib/Commands.cpp
    cmdlib/Commands.h
    cmdlib/Context.cpp
    cmdlib/Context.h
    cmdlib/IConsole.h
    cmdlib/ProcessRunner.cpp
    cmdlib/ProcessRunner.h
    cmdlib/RobotCommand.cpp
    cmdlib/RobotCommand.h
    cmds/AddRobotCmd.cpp
    cmds/AddRobotCmd.h
    cmds/CompileCmd.cpp
    cmds/CompileCmd.h
    cmds/DeleteLogsCmd.cpp
    cmds/DeleteLogsCmd.h
    cmds/DeployCmd.cpp
    cmds/DeployCmd.h
    cmds/DownloadCameraCalibrationsCmd.cpp
    cmds/DownloadCameraCalibrationsCmd.h
    cmds/DownloadLogsCmd.cpp
    cmds/DownloadLogsCmd.h
    cmds/ExitCmd.cpp
    cmds/ExitCmd.h
    cmds/HelpCmd.cpp
    cmds/HelpCmd.h
    cmds/PushConfigCmd.cpp
    cmds/PushConfigCmd.h
    cmds/RestartCmd.cpp
    cmds/RestartCmd.h
    cmds/SCPCmd.cpp
    cmds/SCPCmd.h
    cmds/SSHCmd.cpp
    cmds/SSHCmd.h
    cmds/ShortcutCmd.cpp
    cmds/ShortcutCmd.h
    cmds/ShowCmd.cpp
    cmds/ShowCmd.h
    cmds/ShutdownCmd.cpp
    cmds/ShutdownCmd.h
    cmds/SimCmd.cpp
    cmds/SimCmd.h
    dorsh.cpp
    dorsh.qrc
    dorsh.rc
    icons/dorsh_terminal.icns
    models/Power.cpp
    models/Power.h
    models/Robot.cpp
    models/Robot.h
    models/Team.cpp
    models/Team.h
    tools/Directory.cpp
    tools/Directory.h
    tools/Filesystem.cpp
    tools/Filesystem.h
    tools/Platform.cpp
    tools/Platform.h
    tools/ShellTools.cpp
    tools/ShellTools.h
    tools/Sleeper.h
    tools/StringTools.cpp
    tools/StringTools.h
    ui/CommandLineCompleter.cpp
    ui/CommandLineCompleter.h
    ui/CommandLineEdit.cpp
    ui/CommandLineEdit.h
    ui/Console.cpp
    ui/Console.h
    ui/MainWindow.cpp
    ui/MainWindow.h
    ui/RobotPool.cpp
    ui/RobotPool.h
    ui/RobotView.cpp
    ui/RobotView.h
    ui/SensorWindow.cpp
    ui/SensorWindow.h
    ui/ShortcutBar.cpp
    ui/ShortcutBar.h
    ui/TeamSelector.cpp
    ui/TeamSelector.h
    ui/TeamView.cpp
    ui/TeamView.h
    ui/VisualContext.cpp
    ui/VisualContext.h
)

set_target_properties(${PROJECT_NAME}
    PROPERTIES
        MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME}
        MACOSX_BUNDLE_BUNDLE_GUI_IDENTIFIER "edu.tudo.dorsh"
        MACOSX_BUNDLE_ICON_FILE dorsh_terminal.icns
)
set_source_files_properties(icons/dorsh_terminal.icns PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")

target_include_directories(${PROJECT_NAME} PRIVATE ../..)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        Qt6::Concurrent
        Qt6::Network
        nlohmann_json::nlohmann_json
        Threads::Threads
        GameController
)

if(WIN32)
    target_link_libraries(${PROJECT_NAME}
        PRIVATE
            ws2_32
            winmm
    )
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE TARGET_TOOL)

target_optimize(${PROJECT_NAME} FULL RelWithDebInfo Release)

set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOMOC ON)
set_property(TARGET ${PROJECT_NAME} PROPERTY AUTORCC ON)
