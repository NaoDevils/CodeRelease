#! /usr/bin/env python3
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------------------
# Name:			installRobot
# Info:			Install-script for a new Nao-Robot. Supports only NaoV6
# Author:		Thomas Schulte-Althoff, Dominik Brämer
# Created:		21.01.2019
# Version:		19-08-06
#-------------------------------------------------------------------------------

import argparse
import time
import csv
import getopt
import os
import shutil
import sys
import time
import warnings
from typing import NewType, Tuple

import robotsParser
import robotTools
import teamsParser
from Include import bhumanBase, ssh

os.chdir(os.path.join(os.path.dirname(__file__), '.'))
RobotList = NewType('RobotList', robotsParser.RobotList)

# global ssh connection:
sc: ssh.Connector = ""

def setHostname(newHostname: str):
	"""
	Change the hostname of the robot.
	
	Args:
		newHostname (str): New hostname
	"""
	
	print("Set hostname of NAO")
	sc.exec("echo " + newHostname + " > /tmp/hostname")
	sc.exec("mv /tmp/hostname /etc/hostname", root = True)
	print("... set hostname done.")

def setWiFi(wifiName: str):
	"""
	Set the wifi configuration on the robot.

	Args:
		wifiName (str): ssid of the access point
	"""

	print("Set wifi config")
	if(wifiName == None or wifiName == ""):
		exit("setWiFi: Parameter wifiName is empty!")
	print("-> New WiFi network:", wifiName)
	wifiConfigFile = bhumanBase.networkFiles + wifiName
	print("-> WiFi config-file: ", wifiConfigFile)
	# Check if config files for new wifi network does exist:
	if not (os.path.isfile(wifiConfigFile)):
		exit("WiFi configuration ist not found: " + wifiConfigFile)
	# update wifi:
	sc.xfer(wifiConfigFile, "/home/nao/wifi.config")
	print("... wifi config done.")

def parseNetworkConfig(robotName: str) -> Tuple[str, str, str]:
	"""
	Read the network information of the specified robot from config file.
	
	Args:
		robotName (str): Name of the robot
	
	Returns:
		Tuple[str, str, str]: hostname, lan IP and wlan IP
	"""

	hostname = ""
	ip_lan = ""
	ip_wlan = ""
	# Config file for network:
	networkConfigFile = bhumanBase.robotsDir + robotName + "/network.cfg"
	print("-> Networkconfigfile: ...", networkConfigFile)
	# check if config file exists:
	if not(os.path.isfile(networkConfigFile)):
		exit("Network configuration is not found for:", robotName)
	# Read net-configuration:
	with open(networkConfigFile, newline='') as csvfile:
		reader = csv.reader(csvfile, delimiter='=', quotechar='"')
		for row in reader:
			option = row[0].replace(" ", "")
			value = row[1].replace('"', "").replace(" ", "").replace(";", "")
			if (option == "name" and value != ""):
				hostname = value
			elif (option == "lan" and value != ""):
				ip_lan = value
			elif (option == "wlan" and value != ""):
				ip_wlan = value
			else:
				exit("Unable to parse network config file: ", networkConfigFile)
	print("-> New network configuration is: <hostname> <lan> <wlan> ...", hostname, ip_lan, ip_wlan)
	return(hostname, ip_lan, ip_wlan)

def setNetwork(robotName:str, wifiName:str):
	"""
	Controls which network options have to be changed and setup the network correctly.
	
	Args:
		robotName (str): Name of the robot
		wifiName (str): ssid of the wifi access point used for network connection
	"""

	hostname, _, _ = parseNetworkConfig(robotName)
	# set wifi initial config:
	setWiFi(wifiName)
	# set hostname:
	setHostname(hostname)
	print("Install networking service:")
	print("-> Disable Connman in /lib/systemd/system")
	sc.exec("mount -o remount,rw /", root = True)
	sc.exec("ln -sf /dev/null /lib/systemd/system/connman.service", root = True)
	# transfer file before umount the overlay
	# because file transfer opens a second ssh tunnel
	# which can cause problems after the umount of the overlay
	sc.xfer(bhumanBase.installFiles + "framework/networking.service", "/tmp/networking.service")
	print("-> umount overlay...")
	sc.exec("umount -l /etc", root = True)
	print("-> Delete old networking service...")
	sc.exec("rm /etc/systemd/system/networking.service", root = True)
	print("-> Create new networking service...")
	sc.exec("cp /tmp/networking.service /etc/systemd/system/networking.service", root = True)
	print("-> Disable wpa_supplicant service...")
	sc.exec("systemctl disable wpa_supplicant", root = True)
	sc.exec("systemctl daemon-reload", root = True)
	print("-> Enable new networking service...")
	sc.exec("systemctl enable networking", root = True)
	print("-> restart overlay...")
	sc.exec("systemctl restart etc.mount", root = True)
	print("-> done.")
	print("... network setup done.")

def updateScripts(robotName: str, remove: bool = True):
	"""
	Update all necessary script files on the robot.
	Please add new scripts here.
	
	Args:
		robotName (str): Name of the robot
		remove (bool, optional): Deletes and recreate the script folder on the robot. Defaults to True.
	"""

	print("Update the scripts on Nao (/home/nao/<ScriptDir>)")
	scriptDir = bhumanBase.scriptDir
	if scriptDir == "":
		exit("scriptDir is not set in bhumanBase.py")
	# create script folder:
	if remove or bhumanBase.checkFileOnRobot(sc, scriptDir):
		print('-> Remove old script files and folders...')
		sc.exec("rm -r {}".format(scriptDir))
		sc.exec("mkdir {}".format(scriptDir))

	# copy files:
	print('-> Transfer script files...')
	sc.xfer(bhumanBase.binFiles + "network.sh", scriptDir + 'network.sh')
	sc.xfer(bhumanBase.binFiles + "wifiState.sh", scriptDir + 'wifiState.sh')

	# copy alsa config to tmp:
	print('-> Transfer alsa config...')
	sc.xfer(bhumanBase.installFiles + "alsa.conf", "/tmp/alsa.conf")
	sc.exec("mount -o remount,rw /", root = True)
	print('-> Update alsa config...')
	sc.exec("cp /usr/share/alsa/alsa.conf /usr/share/alsa/alsa.conf.default", root = True)
	sc.exec("mv /tmp/alsa.conf /usr/share/alsa/alsa.conf", root = True)

	# change ROBOT_LAN_IP and ROBOT_WLAN_IP:
	print('-> Get network addresses...')
	_, ip_lan, ip_wlan = parseNetworkConfig(robotName)
	print('-> Update network addresses in network script...')
	sc.exec("sed -i 's/ROBOT_LAN_IP/{ip_lan}/g' /home/nao/scripts/network.sh".format(ip_lan = ip_lan))
	sc.exec("sed -i 's/ROBOT_WLAN_IP/{ip_wlan}/g' /home/nao/scripts/network.sh".format(ip_wlan = ip_wlan))

	# chmod:
	print('-> Make scripts runnable...')
	sc.exec("chmod +x /home/nao/scripts/network.sh")
	sc.exec("chmod +x /home/nao/scripts/wifiState.sh")

	# create folders:
	print('-> Create log folder...')
	sc.exec("mkdir {}".format("log"))

	# change nothing from here!
	# some hint for the readme file on robot:
	readmeMsg = "This folder was generated by installRobot.py. All files could be deleted, if script is executed again. Do not save your files here! Best regards, Mr. Keks."
	sc.exec("echo '" + readmeMsg + "'>{dir}/README".format(dir = scriptDir))
	print('-> done.')

def configRobot():
	"""
	Function which setup the configuration of the robot.
	Please add your own or new files/folders/scripts here
	"""

	print("ConfigRobot...")
	
	# 1: Update autoload.ini:
	print("1: Update autoload.ini:")
	sc.exec("mount -o remount,rw /", root = True)
	print("-> backup autoload.ini")
	sc.exec("mv -n /opt/aldebaran/etc/naoqi/autoload.ini /opt/aldebaran/etc/naoqi/autoload.bak", root = True)
	sc.xfer(bhumanBase.installFiles + "autoloadV6.ini", "/tmp/autoload.ini")
	print("-> Create new autoload.ini")
	sc.exec("mv /tmp/autoload.ini /opt/aldebaran/etc/naoqi/autoload.ini", root = True)
	# sc.exec("systemctl --user restart naoqi")
	print("-> done.")

	# 2: Create empty robocup.conf
	print("2: Create robocup.conf:")
	sc.exec("touch /home/nao/robocup.conf")
	print("-> done.")

	# 3: Install framework-services:
	print("3: Install framework-services:")
	print("-> Create bin directory...")
	sc.exec("mkdir /home/nao/bin")
	print("-> Create logs directory...")
	sc.exec("mkdir /home/nao/logs")
	print("-> Install chronyd...")
	sc.xfer(bhumanBase.binFiles + "chronyd", "/tmp/chronyd")
	sc.exec("cp /tmp/chronyd /home/nao/bin/chronyd")
	sc.exec("chmod +x /home/nao/bin/chronyd")
	print("-> Install chronyc...")
	sc.xfer(bhumanBase.binFiles + "chronyc", "/tmp/chronyc")
	sc.exec("cp /tmp/chronyc /home/nao/bin/chronyc")
	sc.exec("chmod +x /home/nao/bin/chronyc")
	print("-> Create chrony.conf...")
	sc.xfer(bhumanBase.installFiles + "chrony.conf", "/home/nao/chrony.conf")
	print("-> Create chronyd sudoers config...")
	sc.xfer(bhumanBase.installFiles + "chronyd_sudoers_config", "/tmp/chronyd_sudoers_config")
	print("-> Add chronyd sudoers config...")
	sc.exec("cp /tmp/chronyd_sudoers_config /etc/sudoers.d/chronyd_sudoers_config", root = True)
	print("-> Add halt sudoers config...")
	sc.xfer(bhumanBase.installFiles + "halt_sudoers_config", "/tmp/halt_sudoers_config")
	sc.exec("cp /tmp/halt_sudoers_config /etc/sudoers.d/halt_sudoers_config", root = True)
	print("-> Add network sudoers config...")
	sc.xfer(bhumanBase.installFiles + "network_sudoers_config", "/tmp/network_sudoers_config")
	sc.exec("cp /tmp/network_sudoers_config /etc/sudoers.d/network_sudoers_config", root = True)
	print("-> Create systemd user directories...")
	sc.exec("mkdir /home/nao/.config/systemd")
	sc.exec("mkdir /home/nao/.config/systemd/user")
	print("-> Install libs...")
	sc.xfer(bhumanBase.binFiles + "libbsd.so.0.8.2", "/tmp/libbsd.so.0.8.2")
	sc.xfer(bhumanBase.binFiles + "libedit.so.2.0.53", "/tmp/libedit.so.2.0.53")
	sc.exec("cp /tmp/libbsd.so.0.8.2 /usr/lib/libbsd.so.0.8.2", root = True)
	sc.exec("cp /tmp/libedit.so.2.0.53 /usr/lib/libedit.so.2.0.53", root = True)
	print("-> Create softlinks for libs...")
	sc.exec("ln -sf libbsd.so.0.8.2 /usr/lib/libbsd.so.0", root = True)
	sc.exec("ln -sf libedit.so.2.0.53 /usr/lib/libedit.so.2", root = True)
	print("-> Create chronyd service...")
	sc.xfer(bhumanBase.installFiles + "framework/chronyd.service", "/home/nao/.config/systemd/user/chronyd.service")
	print("-> Create bhumand service...")
	sc.xfer(bhumanBase.installFiles + "framework/bhumand.service", "/home/nao/.config/systemd/user/bhumand.service")
	print("-> Create ndevild service...")
	sc.xfer(bhumanBase.installFiles + "framework/ndevild.service", "/home/nao/.config/systemd/user/ndevild.service")
	print("-> Create sensorReaderd service...")
	sc.xfer(bhumanBase.installFiles + "framework/sensorReaderd.service", "/home/nao/.config/systemd/user/sensorReaderd.service")
	sc.exec("systemctl --user daemon-reload")
	print("-> Enable bhumand...")
	sc.exec("systemctl --user enable chronyd.service")
	sc.exec("systemctl --user enable bhumand.service")
	print("-> Enable ndevild...")
	sc.exec("systemctl --user enable ndevild.service")
	print("-> Enable sensorReaderd...")
	sc.exec("systemctl --user enable sensorReaderd.service")
	print('-> Enable chronyd...')
	sc.exec("systemctl --user enable chronyd.service")
	print("-> Create bhuman_init file...")
	sc.xfer(bhumanBase.installFiles + "framework/bhuman_init.sh", "/home/nao/bin/bhuman_init.sh")
	sc.exec("chmod +x /home/nao/bin/bhuman_init.sh")
	print("-> Create ndevil_init file...")
	sc.xfer(bhumanBase.installFiles + "framework/ndevil_init.sh", "/home/nao/bin/ndevil_init.sh")
	sc.exec("chmod +x /home/nao/bin/ndevil_init.sh")
	print("-> Create sensorReader_init file...")
	sc.xfer(bhumanBase.installFiles + "framework/sensorReader_init.sh", "/home/nao/bin/sensorReader_init.sh")
	sc.exec("chmod +x /home/nao/bin/sensorReader_init.sh")
	print("-> done.")

	# 5: Copy ssh key(s):
	# Last step because of some ssh issues
	# if you try to do this at a earlier moment
	print("4: Copy the ssh key(s):")
	sc.exec("mkdir /home/nao/.ssh")
	sc.xfer(bhumanBase.installFiles + "authorized_keys", "/home/nao/.ssh/authorized_keys")
	sc.exec("chmod 700 /home/nao/.ssh")
	sc.exec("chmod 600 /home/nao/.ssh/authorized_keys")
	print("-> done.")

	# change nothing from here!
	print("...config robot done.")


def installChroot(ip):
	"""
	Function which sets up a chroot on the robot.
	Please add modifications in association with the chroot here
	"""
        
	print("check if chroot is already present")
	sc.exec("ls /data/chrt", root = True)
	print(sc.exitStatus)
	if (sc.exitStatus == 0):
		print("chroot is present so remove it")
		sc.xfer(bhumanBase.installFiles + "cleanup_chroot.sh", "/tmp/cleanup_chroot.sh")
		print("unmount dirs")
		sc.exec("bash /tmp/cleanup_chroot.sh", root = True)
		print("remove chroot directory")
		retstr = sc.exec("rm -rf /data/chrt", root = True)
		print(retstr)
		sc.exec("ls /data/chrt", root = True)
		print(sc.exitStatus)
	else:
		print("chroot is not present")
	if (sc.exitStatus == 0):
		exit()

	print("install chroot...")
	print("-> Create temporary transfer directory...")
	sc.exec("mkdir /tmp/installChroot")
	print("-> Transfer chroot...")
	sc.xfer(bhumanBase.installFiles + "chroot.tgz", "/tmp/installChroot/chroot.tgz")
	print("-> Unzip chroot container...")
	sc.exec("tar -xf /tmp/installChroot/chroot.tgz -C /tmp/installChroot", root = True)
	print("-> Unzip chroot files...")
	sc.exec("tar -xf /tmp/installChroot/chroot/chroot.tar -C /data", root = True)
	print("-> Rename chroot folder...")
	sc.exec("mv /data/chroot /data/chrt", root = True)
	print("-> Initialize chroot...")
	sc.exec("bash /tmp/installChroot/chroot/installChroot.sh", root = True)
	print("-> Unzip kernel...")
	sc.exec("tar -xf /tmp/installChroot/chroot/openCLkernel.tar.gz -C /tmp/installChroot/chroot", root = True)
	print("-> Install kernel...")
	sc.exec("/tmp/installChroot/chroot/kernelstuff/install.sh", root = True)

	print("setup chroot service...")
	print("-> Create chroot_init script")
	sc.exec("cp /tmp/installChroot/chroot/chroot_init.sh /home/nao/bin/chroot_init.sh", root = True)
	sc.exec("chmod +x /home/nao/bin/chroot_init.sh")
	print("-> Remount root directory...")
	sc.exec("mount -o remount,rw /", root = True)
	print("-> umount overlay...")
	sc.exec("umount -l /etc", root = True)
	print("-> Create chrootInit service...")
	sc.exec("cp -u /tmp/installChroot/chroot/chrootInit.service /etc/systemd/system/chrootInit.service", root = True)
	print("-> Enable new chroot service...")
	sc.exec("systemctl daemon-reload", root = True)
	sc.exec("systemctl enable chrootInit", root = True)
	print("-> restart overlay...")
	sc.exec("systemctl restart etc.mount", root = True)
	print("... done.")

def createNewRobot(robotList: RobotList, name: str = None):
	"""
	Interactive function which generates all necessary files and folders to add a new robot
	
	Args:
		robotList (RobotList): A list which contains all currently used robots (robots.cfg)
		name (str): Name of robot to add. If set the function is non interactive.
	"""
	if name is None:
		createRobot = input('Robot not found would you like to create a new robot [y/n]: ')
	else:
		createRobot = 'y'

	if createRobot.lower() == 'y' or createRobot.lower() == 'yes':
		if name is None:
			robotName = input('Robot Name: ')
		else:
			robotName = name

		if robotList.getRobotName(robotName) is not None:
			print("Robot name already exist choose another one")
			if name is None:
				createNewRobot(robotList)
			else:
				exit()
		
		robotId = robotList.getNextId(robotName)

		print('Create robot {robotName} with id {robotId}'.format(robotName=robotName, robotId=robotId))
		if name is None:
			inputCorrect = input('Is this correct [y/n]: ')
		else:
			inputCorrect = 'y'

		if inputCorrect.lower() == 'y' or inputCorrect.lower() == 'yes':
			headId, bodyId = bhumanBase.getIdsFromRobot(sc)
			robotList.addRobotToList(robotName, headId, bodyId, robotId, 'V6')
			
			baseDir = os.getcwd()
			robotDir = '{baseDir}/Robots/{robotName}/'.format(baseDir=baseDir, robotName=robotName)
			robotTool = robotTools.RobotTools(robotDir, robotName, robotId)
			robotTool.createRobot()

			robotList.saveRobotListToFile()
		elif inputCorrect.lower() == 'n' or inputCorrect.lower() == 'no':
			print('Please try it again')
			createNewRobot(robotList)
		else:
			print('Please type y or yes to create a new robot otherwise type n or no')
			createNewRobot(robotList)
	elif createRobot.lower() == 'n' or createRobot.lower() == 'no':
		exit()
	else:
		print('Please type y or yes to create a new robot otherwise type n or no')
		createNewRobot(robotList)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
# configure the input arguments:
parser = argparse.ArgumentParser(description='This script install a new robot.')
parser.add_argument('-ip', '--ip', help='actual LAN-IP of the NAO. The Nao must be connected wired')
parser.add_argument('-c', '--chroot', action='store_true', help='install chroot on the robot')
parser.add_argument('-l', '--list', action='store_true', help='show a list of all robots')
parser.add_argument('-a', '--add', dest='robot', help='add a robot with the given name')
parser.add_argument('-d', '--delete', dest='name', help='delete a given robot and corresponding files/folders')
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

if __name__ == "__main__":
	# parse the arguments:
	args = parser.parse_args()

	if args.list:
		robotList = robotsParser.RobotList()
		print(robotList)
		exit()
	if args.name is not None:
		robotList = robotsParser.RobotList()
		team = teamsParser.Team()
		assert robotList.getRobotId(args.name) is not None, 'Can not find robot in list'

		robotList.removeRobotFromList(args.name)
		robotList.saveRobotListToFile()

		team.removePlayer(args.name)
		team.saveTeamsConfig()

		baseDir = os.getcwd()
		robotDir = '{baseDir}/Robots/{robotName}/'.format(baseDir=baseDir, robotName=args.name)
		robotTool = robotTools.RobotTools(robotDir, args.name, robotList.getRobotId(args.name))
		robotTool.removeRobot()

		print('Robot {robotName} successfully removed.'.format(robotName=args.name))
		exit()
	if args.ip is not None:
		if not bhumanBase.checkIpAddress(args.ip):
			exit('{ip} is not a valid ip address'.format(ip = args.ip))
	else:
		exit('No actual IP-Adress is given!')

	print('installRobot-script: Start...')

	# connect to the robot:
	sc = ssh.Connector(args.ip)

	if args.chroot:
		# setup chroot
		installChroot(args.ip)
		print("rebooting")
		sc.exec("reboot", root = True, rebootFlag = True)
		exit()

	robotList = robotsParser.RobotList()
	robotIdentifier = bhumanBase.getHeadIdOrNameFromRobot(sc)
	robot = robotList.getRobotName(robotIdentifier)

	if robot == None:
		if args.robot is not None:
			createNewRobot(robotList, args.robot)
		else:
			createNewRobot(robotList)
		robotIdentifier = bhumanBase.getHeadIdOrNameFromRobot(sc)
		robot = robotList.getRobotName(robotIdentifier)

	# always update the scripts
	updateScripts(robot)

	# set network config:
	print("Set network to default: RC_B")
	setNetwork(robot, "RC_B")

	# install the ndevil setup
	configRobot()

	print("rebooting")
	sc.exec("reboot", root = True, rebootFlag = True)

	exit("installRobot-script: DONE")
