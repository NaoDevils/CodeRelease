#!/usr/bin/python
import naoqi
import sys
from naoqi import ALProxy
memory = ALProxy("ALMemory","127.0.0.1",9559)
value =  memory.getData("Device/SubDeviceList/Battery/Charge/Sensor/Value")
current =  memory.getData("Device/SubDeviceList/Battery/Current/Sensor/Value")
print str(sys.argv[1]) + " " + str(value) + " " + str(current)[:5]
