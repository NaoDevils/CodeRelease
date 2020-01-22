Nao Devils Code Release 2019
=================

This is the 2019 Nao Devils Code Release based on the [B-Human code release 2015](https://github.com/bhuman/BHumanCodeRelease/tree/coderelease2015).

Getting started
---------------

Please refer to the original B-Human Code Release pdf for additional information.

### Building

Make sure to download the NAOqi C++ SDK for Linux from [Aldebaran/Softbank](https://community.ald.softbankrobotics.com/) (account required!), which is required to communicate with the Nao V5. There is a script available, that extracts and copies the various header files to their required directories (`Install/installAlcommon`).
The project uses a custom build tool called [mare](https://github.com/craflin/mare), all marefiles are in the directory `Make/Common`.

Available build targets:
- Nao
  - libbhuman (for V5)
  - libgamectrl (for V5)
  - ndevilsbase (for V6)
  - sensorReader (for V6)
- SimRobot
  - SimRobotCore2
  - SimRobotEditor
  - SimRobotHelp
  - SimulatedNao
    - Controller
    - qtpropertybrowser
    - libqxt
- dorsh

Available build configurations:
- Debug
- Develop
- Relese


It is enough to build the three main targets as they depend on all other ones. `Nao` (cross) compiles the robot's code, `SimRobot` compiles the Simulator and `dorsh` is the deployement tool.

#### Windows

Required software:
  - Windows 10
  - Visual Studio 2019
  - Windows Subsystem for Linux with Ubuntu 18.04 and the following packages:
    - clang-6.0 (later versions upto Clang 9 should also work)
    - ccache
    - make

Mare includes exporting a native project to Visual Studio. You can run the `Make/VS2019/generate.cmd` batch file to generate a Visual Studio solution `NDevils.sln` that can be opened afterwards. Remember to generate new project files each time you add or remove a file from the project, e.g. if you are switching branches.

#### Linux

Required software packages (for Ubuntu 18.04, packages for other distributions may be called different):
- libqt4-dev
- libqt4-opengl-dev
- libqt4-dev-bin
- qt4-dev-tools
- libglew-dev
- libxml2
- libxml2-dev
- glew-utils
- libjpeg-dev
- clang-6.0 (later versions upto Clang 9 should also work)
- cmake

You have multiple options to export the marefiles to project files. You may use your own editor and simple invoke
```
make <target> CONFIG=<config>
```
inside the directory `Make/Linux`, where `<target>` is one of the targets and `<config>` is one of the build configurations.

The generate script in the directory `Make/LinuxQtCreator` will copy a CMakeLists.txt file to the root directoy. After that you may load the project with any CMake compatible IDE. The `CMakeLists.txt` simply grabs all cpp and header files and puts them into a dummy target, so that the IDE will show these files as project files. For each target of the framework, there is a custom target that simply invokes the above make commands as custom commands. We recommend QTCreator in connection with cmake.

### Setting up the Nao

### V5 

Please refer to the B-Human Code Release report, as the process is the same.
Note that we've enabled connecting to the Nao V5 via ssh as root.
If you do not wish to do this, remove lines 48 to 50 in `Install/Files/install`, but be careful, this will break the NTP synchronization while deploying, since root is required to set the time.
Also, the ssh keys provided in this code release are the ones provided by the B-Human Code Release, for security reasons you might want to change these as well.

### V6

Using the Nao V6 requires the RoboCup-only version of the operating system (currently version 2.8.5.11) offering a direct communication to LoLA that is used by our software. Installation instructions can be found in the [documentation](http://doc.aldebaran.com/2-8/software/naoflasher/naoflasher.html).

The install-script for the NAO V6 now requires a valid python3 installation and additionally the python3-pip package paramiko (to handle the ssh connection).
The new install-script simplifies the installation process and managing of the robots. Therefore, it is now possible to add new robots to our infrastructure on the fly while starting the installation process.  
It is also possible to take a look at the robots which are already known by our infrastructure and removing them. This is done all by the same script so that you don't have to switch between some scripts like you have to do for the NAO V5. 
Here are some examples to show you how to use the new install-script and how to fully install our framework infrastucture on the NAO V6.

#### Installation of the framework infrastructure (on a new/unknown robot)
##### Interactive
- Example #1: `python installRobot.py --ip <IpOfTheRobot>`
- Example #2: `py installRobot.py --ip <IpOfTheRobot>`

During the installation process it is recognized that the robot is new or unknown and asked whether the configuration files for this robot should be created and which name the new robot should have.

##### Manual / non-interactive
- Example #1: `python installRobot.py --ip <IpOfTheRobot> --add <NameOfTheRobot>`
- Example #2: `py installRobot.py -ip <IpOfTheRobot> -a <NameOfTheRobot>`
  
During the installation process it is recognized that the robot is new or unknown and the configuration files for the robot with the given name are created.
  
#### Managing robots
##### Show all known robots
- Example #1: `python installRobot.py --list`
- Example #2: `py installRobot.py -l`

This shows a list of all known robots ordered by name. The shown list includes the head ID, body ID, the last octet of the IP (robot ID) and the NAO version.

##### Remove an existing robot
- Example #1: `python installRobot.py --delete <NameOfTheRobot>`
- Example #2: `py installRobot.py -d <NameOfTheRobot>`

This will delete all corresponding files and entries of the given robot in our infrastructure.

### Copying the compiled files

You should use `dorsh` (which is based on B-Human's `bush`) as preferred deploy tool that copies the required data and settings to the robot. (Detailed usage see B-Human Code Release.)

Please note that after a fresh installation, the Nao will not appear as available in dorsh and the LAN device must be manually selected in the upper right corner. During the first transmission, our SensorReader is copied to the Nao, which broadcasts the current status of the robot to dorsh. 
