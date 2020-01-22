#! /usr/bin/env python3
#-------------------------------------------------------------------------------
# Name:			robotParser
# Info:			Robot.cfg parser to convert file into custom python list 
#               and vice versa.
# Author:		Dominik Brämer
# Created:		24.07.2019
# Version:		19-08-05
#-------------------------------------------------------------------------------
from typing import List

from Include import bhumanBase


class Robot:
    """
    Robot container which provides robot name, head ID, body ID, robot ID and version.
    """

    def __init__(self, parsingList: List[str] = None, robotName: str = None, headId: str = None, bodyId: str = None, robotId: str = None, naoVersion: str = None):
        if not parsingList is None:
            self.robotName = parsingList[0]
            self.headId = parsingList[1]
            self.bodyId = parsingList[2]
            self.robotId = parsingList[3]
            self.naoVersion = parsingList[4]
        else:
            self.robotName = robotName
            self.headId = headId
            self.bodyId = bodyId
            self.robotId = robotId
            self.naoVersion = naoVersion

    def getRobotAsList(self) -> List[str]:
        """
        Return all robots information as a simple list of strings.
        
        Returns:
            List[str]: [robotName, headId, bodyId, robotId, naoVersion]
        """
        return [self.robotName, self.headId, self.bodyId, self.robotId, self.naoVersion]

    def __lt__(self, other):
        return self.robotName < other.robotName

    def __contains__(self, key):
        if self.robotName == key or self.headId == key or self.bodyId == key or self.robotId == key:
            return True
        else:
            return False
    
    def __str__(self):
        result = '{{ name = {self.robotName}; headId = {self.headId}; bodyId = {self.bodyId}; id = {self.robotId}; naoVersion = {self.naoVersion}; }}'.format(self=self)
        return result

    def __repr__(self):
        return self.__str__()

class RobotList:
    """
    List of robots generated from and to ./NDevils2015/Config/Robots/robots.cfg.
    """

    def __init__(self):
        #filePath = './NDevils2015/Config/Robots/robots.cfg'
        filePath = bhumanBase.robotsFile
        robotsFile = open(filePath)
        numLines = sum(1 for line in open(filePath))

        self.robots = []
        self.maxLengthName = 0
        self.maxLengthID = 0
        for index, line in enumerate(robotsFile):
            if index < 2 or index > numLines - 2:
                continue

            parsingList = []
            word = ''
            parsing = False
            for char in line:
                if char == '=':
                    parsing = True
                    continue
                if char == ';':
                    parsing = False
                    parsingList.append(word)
                    word = ''
                if parsing and char != ' ' and char != '"':
                    word = word + char
            if len(parsingList[0]) > self.maxLengthName:
                        self.maxLengthName = len(parsingList[0])
            if len(parsingList[1]) > self.maxLengthID:
                        self.maxLengthID = len(parsingList[1])
            self.robots.append(Robot(parsingList=parsingList))
    
    def addRobotToList(self, robotName: str, headId: str, bodyId: str, robotId: str, naoVersion: str):
        """
        Add a robot to the list.
        
        Args:
            robotName (str): Name of the robot
            headId (str): Head ID of the robot
            bodyId (str): Body ID of the robot
            robotId (str): Last octet from LAN and WLAN IP of the robot
            naoVersion (str): The version of the robot e.g. V6
        """

        assert headId != bodyId, 'headId and bodyId can not be the same!'
        assert not any(robotName in robot for robot in self.robots), 'Robot with robotName: ' + robotName + ' already exists!'
        assert not any(headId in robot for robot in self.robots), 'Robot with headId: ' + headId + ' already exists!'
        assert not any(bodyId in robot for robot in self.robots), 'Robot with bodyId: ' + bodyId + ' already exists!'
        assert not any(robotId in robot for robot in self.robots), 'Robot with robotId: ' + robotId + ' already exists!'

        robot = Robot(robotName=robotName, headId=headId, bodyId=bodyId, robotId=robotId, naoVersion=naoVersion)
        self.robots.append(robot)

    def removeRobotFromList(self, robotName: str) -> bool:
        removeRobot: Robot = None
        for robot in self.robots:
            if robot.robotName == robotName:
                removeRobot = robot
                break
        self.robots.remove(removeRobot)

    def saveRobotListToFile(self):
        """
        Convert RobotList to robot.cfg format and saves the result in file.
        """
        fileHeader = '//WARNING: Do not edit this file manually, use the script addRobotIds instead!\n'

        fileBody = 'robotsIds = [\n'
        for index, robot in enumerate(self.robots):
            if index < len(self.robots) - 1:
                fileBody = fileBody + '  ' + str(robot) + ',\n'
            else:
                fileBody = fileBody + '  ' + str(robot) + '\n'
        fileBody = fileBody + '];' + '\n'

        fileContent = fileHeader + fileBody

        robotsFile = open(bhumanBase.robotsFile, 'w')
        robotsFile.write(fileContent)
    
    def getRobotId(self, key: str) -> str:
        """
        Returns the robot ID.
        
        Args:
            key (str): One of the following identifiers: robotName, headId, bodyId or robotId
        
        Returns:
            str: Last octet from LAN and WLAN IP of the robot
        """

        for robot in self.robots:
            if key in robot:
                return robot.robotId
    
    def getRobotName(self, key: str) -> str:
        """
        Returns the robot name.
        
        Args:
            key (str): One of the following identifiers: robotName, headId, bodyId or robotId
        
        Returns:
            str: The name of the robot
        """

        for robot in self.robots:
            if key in robot:
                return robot.robotName
    
    def getNaoVersion(self, key: str) -> str:
        """
        Returns the robot version.
        
        Args:
            key (str): One of the following identifiers: robotName, headId, bodyId or robotId
        
        Returns:
            str: The version of the robot
        """

        for robot in self.robots:
            if key in robot:
                return robot.naoVersion

    def getHeadId(self, key: str) -> str:
        """
        Returns the head ID of the robot.
        
        Args:
            key (str): One of the following identifiers: robotName, headId, bodyId or robotId
        
        Returns:
            str: The head ID of the robot
        """

        for robot in self.robots:
            if key in robot:
                return robot.headId

    def getBodyId(self, key: str) -> str:
        """
        Returns the body ID of the robot.
        
        Args:
            key (str): One of the following identifiers: robotName, headId, bodyId or robotId
        
        Returns:
            str: The body ID of the robot
        """

        for robot in self.robots:
            if key in robot:
                return robot.bodyId
    
    def getNextId(self, robotName: str) -> str:
        """
        Returns the next available robot id. Prefers to use first letter of robot name converted to number (A=100,B=101,...).
        
        Args:
            robotName (str): Name of the robot.
        
        Returns:
            str: Next available robot id.
        """

        excludeIds = [122] # ID of the NTP/DHCP Server
        for robot in self.robots:
            excludeIds.append(int(robot.robotId))

        freeId = 100 # lowest possible robot id
        
        preferedRobotId = freeId + ord(robotName[0].upper()) - 65
        if preferedRobotId in excludeIds: 
            while freeId in excludeIds:
                freeId = freeId + 1
        else:
            freeId = preferedRobotId
        
        assert freeId >= 100 and freeId < 255, 'No more free IDs available'

        return str(freeId)

    def __addSpace(self, text: str, minField: int=25) -> str:
        """
        Fill the given text with spaces to reach the min field size.
        
        Args:
            text (str): String which should be filled.
            minField (int, optional): Minimum text size as goal. Defaults to 25.
        
        Returns:
            str: [description]
        """
        spaceToAppend = minField - len(text) 
        result = '' 
        if spaceToAppend <= 0:
            result = text
        else:
            result = text + ' '*spaceToAppend
        return result 

    def __str__(self):
        self.robots.sort()
        message = [['Name', 'Head ID', 'Body ID', 'ID', 'NAO Version'], 
                    ['-'*self.maxLengthName, '-'*self.maxLengthID, '-'*self.maxLengthID, '-'*3, '-'*2]]
        for robot in self.robots:
            message.append(robot.getRobotAsList())

        result = ''
        for line in message:
            for index, elem in enumerate(line):
                if index == 0:
                    result = result + self.__addSpace(elem, self.maxLengthName + 5)
                elif index == 1:
                    result = result + self.__addSpace(elem, self.maxLengthID + 5)
                elif index == 2:
                    result = result + self.__addSpace(elem, self.maxLengthID + 5)
                elif index == 3:
                    result = result + self.__addSpace(elem, 8)
                else:
                    result = result + self.__addSpace(elem, 7)
            result = result + '\n'
        return result

    def __repr__(self):
        return self.__str__()