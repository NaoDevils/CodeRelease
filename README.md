Nao Devils Code Release 2016
=================

This is the 2016 Nao Devils Code Release based on the [B-Human code release](https://github.com/bhuman/BHumanCodeRelease) 2015.

Getting started
---------------

Please refer to the original B-Human Code Release pdf for additional information.
The pdf is included in this repository (`BHumanCodeRelease2015.pdf`).

### Building

Make sure to download the NAOqi C++ SDK from [Aldebaran/Softbank](https://community.ald.softbankrobotics.com/) (account required!). There is a script available, that extracts and copies the various header files to their required directories (`Install/installAlcommon`). The project uses a custom build tool called [mare](https://github.com/craflin/mare), all marefiles are in the directory `Make/Common`.

Available build targets:
- Nao
  - libbhuman
  - libgamectrl
- SimRobot
  - SimRobotCore2
  - SimRobotEditor
  - SimRobotHelp
  - SimulatedNao
    - Controller
    - qtpropertybrowser
    - libqxt
- bush

Available build configurations:
- Debug
- Develop
- Relese


It is enough to build the three main targets as they depend on all other ones. `Nao` (cross) compiles the robot's code, `SimRobot` compiles the Simulator and `bush` is the deployement tool by B-Human.

#### Windows

Required software:
  - Visual Studio 2015 +
  - Cygwin x64 with rsync, openssh, ccache and clang (3.8 at the time of this release, there is no gurantee the code will compile with later releases of clang)

Mare includes exporting a native project to Visual Studio. You can run the `Make/VS2015/generate.cmd` batch file to generate Visual Studio solution and project files. Remember to generate new project files each time you add or remove a file from the project, e.g. if you are switching branches.

#### Linux

Required software:
- qt4-dev-tools
- libglew-dev
- libjpeg-dev
- clang (3.7 and 3.8 are compatible, other versions might be)
- libxml-dev

You have multiple options to export the marefiles to project files. You may use your own editor and simple invoke
```
make <target> CONFIG=<config>
```
inside the directory `Make/Linux`, where `<target>` is one of the targets and `<config>` is one of the build configurations.

Note that CodeLite and NetBeans fail to expand some of the new preprocessor macros and streaming functions. This will result in poor auto completion. That is why we've added a CMakeLists.txt inside `Make/LinuxCMake`. The generate script in this directory will copy this file to the root directoy. After that you may load the project with any CMake compatible IDE. The `CMakeLists.txt` simply grabs all cpp and header files and puts them into a dummy target, so that the IDE will show these files as project files. For each target of the framework, there is a custom target that simply invokes the above make commands as custom commands. We recommend QTCreator or kDevelop 5 in connection with cmake.

### Setting up the Nao

Please refer to the B-Human Code Release report, as the process is the same.
Note that we've enabled connecting to the nao via ssh as root.
If you do not wish to do this, remove lines 44 and 45 in `Install/Files/install`, but be careful, this will break the NTP synchronization while deploying, since root is required to set the time.
Also, the ssh keys provided in this code release are the ones provided by the B-Human Code Release, for security reasons you might want to change these as well.

### Copying the compiled files

You can either use the `Make/Common/copyfiles` (or the OS specific links to the script inside the `Make/<OS/IDE>` folders) script or `bush` (detailed usage see B-Human Code Release).
Note, that we have change the way that the robots synchronize time with each other. Instead of transforming timestamps in network packages from other robots to local time, we synchronize the time with a NTP server when the robot is deployed. You need to change the IP Adress of the server inside the `Make/Common/copyfiles` script, line 53.
