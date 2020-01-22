# !/usr/local/bin/python
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------------------
# Name:			setNetwork.py
# Info:				Set the network configuration on the Nao.
#						Supports only NaoV6.
# Author:			Dominik Braemer, Thomas Schulte-Althoff
# Created:		20.11.2018
# Version:		18-11-20
#-------------------------------------------------------------------------------

import dbus
import sys
bus = dbus.SystemBus()

def help():
	"""Show help for this script"""
	print("connman_config.py wifi <IP> <NetMask>")
	print("connman_config.py eth <APNProfile>")

def setWifi(apnProfile):
	"""Initialize connman and prepare all 
	for an fixed IP address"""

	# Scan for available wifi networks
	technologyPath = "/net/connman/technology/wifi"
	technology = dbus.Interface(bus.get_object("net.connman", technologyPath),"net.connman.Technology")
	technology.Scan()

	# Retrieve all available connection points (wifi networks etc)
	managerPath = "/"
	manager = dbus.Interface(bus.get_object("net.connman", managerPath),"net.connman.Manager")
	services = manager.GetServices()
	
	# Lookup all connection points for a given Name (wired stands for ethernet)
	# to get the right entriepoint for the connman dbus api
	wifiPath = ""
	ethPath = ""
	for service in services:
		serviceProp = service.__getitem__(1)
		serviceName = serviceProp.get(dbus.String(u'Name'))
		
		if serviceName.__contains__(apnProfile):
			objectPath = service.__getitem__(0)
			wifiPath = objectPath.strip()
			
	assert(wifiPath != ""), "Access Point should be in range for first connection"
	
	# Try to connect to the defined wifi network and load the right connman wifi config 
	try:
		wifiService = dbus.Interface(bus.get_object("net.connman", wifiPath),"net.connman.Service")
		wifiService.Connect()
	except:
		print("wifi probably already connected")

def setEth(ipAddr, netmask):
	managerPath = "/"
	manager = dbus.Interface(bus.get_object("net.connman", managerPath),"net.connman.Manager")
	services = manager.GetServices()
	ethPath = ""
	for service in services:
		serviceProp = service.__getitem__(1)
		serviceName = serviceProp.get(dbus.String(u'Name'))
		if serviceName.__contains__("Wired"):
			objectPath = service.__getitem__(0)
			ethPath = objectPath.strip()
			
	# Set the fixed IP for ethernet 
	try:	
		ethService = dbus.Interface(bus.get_object("net.connman", ethPath),"net.connman.Service")
		#ethData = { "Method": "manual", "Address": "192.168.101." + robotID, "Netmask": "255.255.255.0" }
		ethData = { "Method": "manual", "Address": ipAddr, "Netmask": netmask }
		ethService.SetProperty('IPv4.Configuration', dbus.Dictionary(ethData, signature='sv'))
	except:
		print("eth config error")

""" - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - """

def main():
	if len(sys.argv) < 3:
		help()
	else:
		if (sys.argv[1] == "wifi"):
			setWifi( sys.argv[2])
		elif (sys.argv[1] == "eth"):
			setEth(sys.argv[2], sys.argv[3])
		else:
			help()

if __name__ == '__main__':
	main()
