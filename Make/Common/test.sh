#!/bin/bash

while read line; do
    echo $line
    if [ "$line" == "Goal by blue" ]; then
        echo "Test succeeded"
        killall Build/Linux/SimRobot/Develop/SimRobot Xvfb
        exit 0
    fi
done < <(timeout 180 xvfb-run Build/Linux/SimRobot/Develop/SimRobot Config/Scenes/TestScene.ros2)

>&2 echo "Robot did not score a goal!"
>&2 echo "Check log output and open TestScene in simulator."
exit 1
