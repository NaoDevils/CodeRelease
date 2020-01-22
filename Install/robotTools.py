#! /usr/bin/env python3
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------------------
# Name:			robotTools
# Info:			Utility for the install robot script to add new robots.
# Author:		Dominik Brämer
# Created:		24.07.2019
# Version:		19-07-25
#-------------------------------------------------------------------------------
import os, shutil
from Include import bhumanBase

class RobotTools:
    """
    Helper class which provides functions to generate all necessary files and folders for a new robot.
    """
    
    def __init__(self, robotDir: str, robotName: str, robotId: str):
        self.robotId = robotId
        self.robotDir = robotDir
        self.robotName = robotName
        self.robotConfigDir = bhumanBase.robotsDir + robotName
        self.calibrationDir = bhumanBase.robotsDir + robotName + '/' + robotName

    def createRobot(self):
        """
        Generates all necessary files at once.
        """

        self._createRobotDirectory()
        self._createRobotConfigDirectory()
        self._createHostname()
        self._createRobotCalibrationDirectory()
        self._createBushConfig()
        assert self._checkFiles(), 'Some robot files are missing!' 
        print('Robot files were generated successfully')

    def removeRobot(self):
        """
        Deletes all robot directories.
        """
        
        self._deleteRobotDirectories()

    def _checkFile(self, filepath: str, emptyCheck: bool = True) -> bool:
        """
        Checks if the file or directory is available or not empty.
        
        Args:
            filepath (str): Path to check.
            emptyCheck (bool, optional): Check for emptiness. Defaults to True.
        
        Returns:
            bool: If the given path points to a file than True if file is available or not empty.
                  If it points to a directory than True if the directory is available.
                  Otherwise False.
        """

        if os.path.isfile(filepath):
            if emptyCheck:
                return os.stat(filepath).st_size > 0
            return True
        if os.path.isdir(filepath):
            return True
        return False

    def _checkFiles(self) -> bool:
        """
        Checks if all needed files for the robot configuration are available.
        
        Returns:
            bool: True if all files are available otherwise False.
        """

        if self._checkFile(self.robotDir) and self._checkFile(self.robotDir + 'hostname') and self._checkFile(self.robotConfigDir):
            return True
        else:
            return False
        
    def _createRobotDirectory(self):
        """
        Creates the robot directory.
        """

        print('Robots-Directory:', self.robotDir)
        if os.path.isdir(self.robotDir):
            shutil.rmtree(self.robotDir)
        os.mkdir(self.robotDir)
        
    def _createRobotConfigDirectory(self) -> bool:
        """
        Creates the config directory.

        Returns:
            bool: True if the directory was created and False if it already exist.
        """

        print('Configuration-Directory:', self.robotConfigDir)
        if not os.path.isdir(self.robotConfigDir):
            os.mkdir(self.robotConfigDir)
            return True
        else:
            return False
            
    def _deleteRobotDirectories(self):
        """
        Deletes robot directories.
        """

        print('Delete all robot directories.')
        shutil.rmtree(self.robotDir, ignore_errors=True)
        shutil.rmtree(self.robotConfigDir, ignore_errors=True)
        
    def _createHostname(self) -> bool:
        """
        Creates local hostname file for the robot.
        
        Returns:
            bool: True if the file was succsessfuly created otherwise False.
        """

        hostnameFile = self.robotDir + 'hostname'
        print('Hostnamefile:', hostnameFile)
        with open(hostnameFile, 'w') as f:
            f.write(self.robotName)
        return self._checkFile(hostnameFile)
        
    def _createRobotCalibrationDirectory(self, delete: bool = False):
        """
        Creates the calibration directory.
        
        Args:
            delete (bool, optional): Delete and recreate the calibration directory. Defaults to False.
        """

        print('Calibration-Directory:', self.calibrationDir)
        if os.path.isdir(self.calibrationDir):
            if delete:
                shutil.rmtree(self.calibrationDir)
                os.mkdir(self.calibrationDir)
        else:
            os.mkdir(self.calibrationDir)

    def _createBushConfig(self):
        """
        Creates nessesary config files for bush.
        """

        teamId = 12
        networkConfig = self.robotConfigDir + '/network.cfg'
        content = 'name = "{name}";\nlan = "192.168.101.{id}";\nwlan = "10.0.{teamId}.{id}";'.format(name=self.robotName, id=self.robotId, teamId=teamId)
        with open(networkConfig, 'w') as f:
            f.write(content)