# How to install joystick drivers

**Drivers required for a gamepad:**
* Copy `joydev.ko`, `ff-memless.ko` and `evdev.ko` from `Install/Files/kernel/2.6.33.9-rt31-aldebaran-rt/kernel/drivers/input` to `/lib/module/<kernel-version>/kernel/drivers/input`.
* Copy `xpad.ko` from `Install/Files/kernel/2.6.33.9-rt31-aldebaran-rt/kernel/drivers/input/joystick` to `/lib/module/<kernel-version>/kernel/drivers/input/joystick`.
* Run `depmod -A` to search for new drivers.

The bash script `scp_joystick_drivers.sh` copies all required drivers to the robot and runs `depmod -A` per ssh. Execute the script in this directory (`Install/files/joystick_drivers/`) and enter the IP address of the robot as argument:
`./scp_joystick_drivers.sh 192.168.101.100`

**Test gamepad drivers:**
* Copy `joystick_test` somewhere on the Nao.
* Connect the Nao to the gamepad.
* Run `joystick_test`.
* If successfully connected this tool prints number and state of changed buttons/axes.
