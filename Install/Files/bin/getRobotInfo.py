# !/usr/local/bin/python
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------------------
# Name:			getRobotInfo
# Info:				Reads some informations from the robot.
#						Supports only NaoV6.
# Author:			Thomas Schulte-Althoff
# Created:		20.11.2018
# Version:		18-11-20
#-------------------------------------------------------------------------------

import sys, os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append("/opt/aldebaran/lib/python2.7/site-packages/") # import qi only after sys.path.append
import qi

# read the head id:
def readHeadID():
	try:
		app = qi.Application(sys.argv)
		app.start()
		almemory = app.session.service("ALMemory")
		id = almemory.getData("RobotConfig/Head/FullHeadId")
		print(id)
		exit(0)
	except RuntimeError, e:
		print("Fail")
		exit(1)

# read the body id:
def readBodyID():
	try:
		app = qi.Application(sys.argv)
		app.start()
		almemory = app.session.service("ALMemory")
		id = almemory.getData("Device/DeviceList/ChestBoard/BodyId")
		print(id)
		exit(0)
	except RuntimeError, e:
		print("Fail")
		exit(1)

# User informations:
def help():
	print("possible arguments are: headId, bodyId, ")

""" - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - """

if __name__ == "__main__":
	if len(sys.argv) < 2:
		help()
	else:
		if (sys.argv[1] == "headId"):
			readHeadID()
		if (sys.argv[1] == "bodyId"):
			readBodyID()
			
