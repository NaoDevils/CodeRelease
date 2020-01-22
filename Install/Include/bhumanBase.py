#! /usr/bin/env python3
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------------------
# Name:			bhumanBase
# Info:			Base-module for the python-scripts. Supports only NaoV6
# Author:		Thomas Schulte-Althoff, Dominik Brämer
# Created:		20.11.2018
# Version:		19-09-24
#-------------------------------------------------------------------------------

import socket
import os
from typing import Tuple

# Workaround for pylint to use type hints
try:
	import ssh
except:
	from Include import ssh

# Folders and Files on repo:
configDir = os.path.dirname(__file__) + "/../../Config/"
installDir = os.path.dirname(__file__) + "/../../Install/"
robotsDir = "{configDir}Robots/".format(configDir = configDir)
robotsFile = "{robotsDir}robots.cfg".format(robotsDir = robotsDir)
teamsFile = "{configDir}teams.cfg".format(configDir = configDir)
installFiles = "{installDir}Files/".format(installDir = installDir)
binFiles = "{installFiles}bin/".format(installFiles = installFiles)
networkFiles = "{installDir}Network/Profiles/".format(installDir = installDir)
# for scripts on Nao at /home/nao/<scriptDir>
scriptDir = "/home/nao/scripts/"

# init user and pass of the robot:
username = "nao"
password = "nao"

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def checkIpAddress(IPAddress: str) -> bool:
	"""
	Checks if the given IP address is valid.
	
	Args:
		IPAddress (str): IP address.
	
	Returns:
		bool: True if IP address is valid otherwise False.
	"""

	try:
		socket.inet_aton(IPAddress)
		return True
	except socket.error:
		return False

def checkFileOnRobot(sc: ssh.Connector, filename: str, debug=False) -> bool:
	"""
	Checks if file on the robot exists.
	
	Args:
		sc (sshConnector): sshConnector with connection to the robot.
		filename (str): Path to the file.
		debug (bool, optional): Activate debug information. Defaults to False.
	
	Returns:
		bool: True if the file exists otherwise False.
	"""

	readCmd = sc.exec(" if [ -e '{}' ]; then echo 1; else echo 0; fi".format(filename)).replace("\n", "")
	if debug:
		print("checkFileOnRobot='{readCmd}'/filename={filename}".format(readCmd=readCmd, filename=filename))
	if str(readCmd) == str(1):
		return True
	else:
		return False

def getHeadIdOrNameFromRobot(sc: ssh.Connector) -> str:
	"""
	Retrieve the head ID or the robot name from the robot.
	
	Args:
		sc (sshConnector): sshConnector with connection to the robot.
	
	Returns:
		str: Contains the robot name if it fails to retrieve the head ID because than the robot is 
		     usually already running in robocup mode. Otherwise it returns the head ID.
	"""

	if checkFileOnRobot(sc, '/home/nao/robocup.conf'):
		returnedLines = sc.exec("cat /etc/hostname")
		name = returnedLines.replace("\n", "")
		return name
	else:
		tmpPath="/tmp/NaoInstall"
		sc.exec("rm -Rf {}".format(tmpPath))
		sc.exec("mkdir -p {}".format(tmpPath))
		sc.xfer(os.getcwd() + "/Files/bin/getRobotInfo.py", tmpPath + "/getRobotInfo.py")
		returnedLines = sc.exec("/usr/bin/python /tmp/NaoInstall/getRobotInfo.py headId", onlyFirstLine=True)
		headId = returnedLines.replace("\n", "")
		return headId

def getIdsFromRobot(sc: ssh.Connector) -> Tuple[str, str]:
	"""
	Retrieves the head and body ID of the robot.
	
	Args:
		sc (sshConnector): sshConnector with connection to the robot.
	
	Returns:
		Tuple[str, str]: head ID, body ID
	"""
	print("Loading ids from Robot")
	tmpPath="/tmp/NaoInstall"
	print("Create temppath")
	sc.exec("rm -Rf {}".format(tmpPath))
	sc.exec("mkdir -p {}".format(tmpPath))
	# Copy files:
	sc.xfer("Files/bin/getRobotInfo.py", tmpPath + "/getRobotInfo.py")
	returnedLines = sc.exec("/usr/bin/python /tmp/NaoInstall/getRobotInfo.py headId", onlyFirstLine=True)
	headId = returnedLines.replace("\n", "")
	returnedLines = sc.exec("/usr/bin/python /tmp/NaoInstall/getRobotInfo.py bodyId", onlyFirstLine=True)
	bodyId = returnedLines.replace("\n", "")
	if headId == "Fail":
		exit("Failed reading the headId!")
	if bodyId == "Fail":
		exit("Failed reading the bodyId!")
	return(headId, bodyId)