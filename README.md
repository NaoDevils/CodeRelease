# Nao Devils Code Release 2022

This is the Nao Devils Code Release 2022, originally based on [B-Human Code Release 2015](https://github.com/bhuman/BHumanCodeRelease/tree/coderelease2015).

## Installation

[Windows 10/11](#windows-1011), Linux (e.g., [Ubuntu 22.04](#ubuntu-2204)) and [macOS](#mac) (experimental) are supported natively.

Build dependencies:
* Microsoft Visual C++ >= 14.34 (Visual Studio 2022 17.4)
* Clang >= 14
* CMake >= 3.20
* [Conan](https://conan.io/) >= 1.52

### Windows 10/11

#### Notes

* [Visual Studio](https://visualstudio.microsoft.com/de/vs/) is the recommended IDE on Windows. Other CMake compatible IDEs such as [JetBrains CLion](https://www.jetbrains.com/de-de/clion/) and [VS Code](https://code.visualstudio.com/) (and possibly more) are also supported. Nevertheless, at least the Microsoft Visual C++ compiler included in [Visual Studio build tools](https://visualstudio.microsoft.com/de/downloads/) is required in this case to build on Windows.
* We recommend at least 40 GiB of free disk space.

#### Installation

You can choose between
* a **[basic installation](#basic-installation)** that includes the robot framework and everything you need to develop and run the simulation and
* a **[full installation](#full-installation)** that also allows to cross-compile the framework for NAO and requires the installation and configuration of *Windows Subsystem for Linux* (WSL).

##### Basic installation

* Install [Python](https://www.python.org/downloads/) and **make sure to check "Add Python to PATH" during setup**.

* Install [Conan](https://conan.io/downloads.html) package manager using `pip install conan` on the command line.

* Install Microsoft Visual Studio:
  * Download [Visual Studio Community 2022 from Microsoft](https://visualstudio.microsoft.com/de/vs/).
  * During setup, **select "Desktop development with C++"**!

* Start VisualStudio and choose "Clone a repository".
  * Repository location: <https://github.com/NaoDevils/CodeRelease.git>
* After that, check the console output at the bottom of the window (may be minimized) and wait until message "CMake generation finished" appears.
* Build simulator. (Menu Build => Build All)
* In the upper toolbar, select **SimRobot.exe** as startup item and run the application.
* See [Testing](#Testing) section on how to load a scene and run the simulation.

##### Full installation

* Complete the basic installation steps above.

* Install Windows Subsystem for Linux (**WSL 1**) running Ubuntu 22.04:
  * The exact steps vary depending on your system version and its previous state.
  * Open a command prompt and execute:
    ```
    wsl --set-default-version 1
    wsl --install -d Ubuntu-22.04
    ```
  * **Notes**:
    * For the first installation, it may not be possible to set the default WSL version to 1 beforehand. In this case, execute the install command first to enable the Windows feature and try it again afterwards.
    * The first WSL installation may require a system reboot. After rebooting, execute the install command again to continue the installation.
    * WSL1 and WSL2 both work, however WSL1 is highly recommended due to frequent Windows file system acccess.
    * You can check the currently used WSL version using `wsl --list --verbose`.
    * You can also convert an existing distribution from WSL 2 to WSL 1 afterwards using `wsl --set-version Ubuntu-22.04 1`.
* Open Ubuntu bash if not opened automatically (type `bash` in command line).
* Update system and install packages:

  ```
  sudo apt update
  sudo apt dist-upgrade
    
  sudo apt install --no-install-recommends clang clang-14 ccache cmake make git ninja-build python3-pip pkg-config clang-format dos2unix
  pip3 install conan
  ```
* In Visual Studio, select **Dorsh.exe** as startup item and run the application.
* Select the checkbox of any robot and click "deploy" in the lower toolbar.
* Check the console output and make sure everything compiles correctly.
* If you are not connected to a robot yet, the command will fail at the end with an error message "Antman is not reachable".

### Ubuntu 22.04

#### Notes

* You can use any CMake compatible IDE you want. [JetBrains CLion](https://www.jetbrains.com/de-de/clion/) is recommended, but others will probably also work.

#### Installation

* Install required packages:

  ```
  sudo apt install --no-install-recommends clang clang-14 ccache cmake make git ninja-build python3-pip pkg-config clang-format dos2unix libgl-dev libglu1-mesa-dev libopengl-dev libegl-dev libasound-dev
  pip3 install conan
  
  # make conan available in path (login again should also work)
  source ~/.profile
  ```


* Afterwards, you can open `CMakeLists.txt` in the repository's root directory using your preferred IDE. Modern IDEs like CLion are able to import configure and build options from `CMakePresets.json`.
* If you would like to compile using the command line, change to the repository's root directory and execute:

  ```
  export CC=clang CXX=clang++ # use Clang compiler if not system default
  cmake --preset "simulator-develop"
  cmake --build --preset "simulator-develop"
  ```
* The initial configuration may take some time depending on the number of Conan packages that are not available precompiled for your platform and have to be compiled from source.
* After compilation, you can start the **SimRobot** binary in `Build/simulator-develop`.
* See [Testing](#Testing) section on how to load a scene and run the simulation.
* To cross-compile for Nao, execute:

  ```
  cmake --preset "nao-develop"
  cmake --build --preset "nao-develop"
  ```

### Mac

Mac support (Intel and Apple M1/M2) is in an experimental state and has been tested using VSCode and CLion, other IDEs may or may not work. Cross compilation for Nao is currently not supported!

* First, install [Homebrew](https://brew.sh). 
* Then, installed some basic requirements for development:

```
brew install cmake conan ninja
```

* You are now ready to build SimRobot as described for Linux using CMake.

### Testing

* Run SimRobot and click *Open*.
* Change to the framework directory and open `Config/Scenes/TwoPlayers.ros2`.
* In *Scene Graph* window, double click on *RoboCup* and on *Console* and arrange both windows as you like.
* Focus the console window and enter `gc playing`.
* Two robots should start playing against each other now.

## Flash a robot

Create a [Nao Devils system image](https://github.com/NaoDevils/NaoImage) using the `generate_naodevils.sh` script. You can either
  * generate a ready-to-run version of our robot software (pass this framework directory using the `-f` parameter) or
  * generate a base version that only contain the Ubuntu base image and has to be deployed using Dorsh later (omit the `-f` parameter).

## Deploy a robot

If you want to deploy/update a robot without re-flashing it everytime, you have to configure it first:
1. Connect the robot to a DHCP enabled network via Ethernet.
2. The robot should say its IP address automatically.
3. Call `Install/addRobot.sh <IP address> <Name> <Robot ID> <Team ID>` from a Linux (e.g., WSL) command prompt and replace the placeholders accordingly. This will change the robot's IP address to `10.0.<Team ID>.<Robot ID>` and `10.1.<Team ID>.<Robot ID>` for LAN and WLAN, respectively.
4. Open Dorsh, move the robot to any player number, select it, and click deploy. (Note: If the robot does not contain our robot software yet and was flashed using the base image, select "Device: LAN" in the upper right corner manually for the first time.)

Steps 1-3 are only required for unknown robots that are not listed in `Config/robots.cfg`.
