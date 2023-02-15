#!/bin/bash

while read line; do
    echo $line
    if [ "$line" == "Goal by blue" ]; then
        echo "Test succeeded"
        killall -q Build/simulator-develop/SimRobot
        exit 0
    fi
done < <(
    timeout --foreground 300 Build/simulator-develop/SimRobot Config/Scenes/TestScene.ros2 -platform offscreen
    killall -q Build/simulator-develop/SimRobot Xvfb
)

>&2 echo "Robot did not score a goal!"
>&2 echo "Check log output and open TestScene in simulator."
exit 1
