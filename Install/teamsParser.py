#! /usr/bin/env python3
#-------------------------------------------------------------------------------
# Name:			teamsParser
# Info:			Teams.cfg parser to convert file into custom python list 
#               and vice versa.
# Author:		Dominik Brämer
# Created:		24.09.2019
# Version:		24-09-19
#-------------------------------------------------------------------------------
from Include import bhumanBase

class Team:
    """
    Team container which provides all team variables.
    """

    def __init__(self):
        #filePath = './NDevils2015/Config/teams.cfg'
        filePath = bhumanBase.teamsFile
        teamsFile = open(filePath)
        numLines = sum(1 for line in open(filePath))

        self.configMap: dict = {}
        listCache = []
        parsingList = False
        listKey = ''
        for index, line in enumerate(teamsFile):
            if index < 2 or index > numLines - 3:
                continue

            value = ''
            key = ''
            parsingConfigValue = False
            if not parsingList:
                for char in line:
                    if not parsingConfigValue:
                        if char == '=':
                            parsingConfigValue = True
                            continue
                        if char != ' ' and char != '"':
                            key = key + char
                    else:
                        if char == ';':
                            parsingConfigValue = False
                            self.configMap[key] = value
                            value = ''

                        if parsingConfigValue and char == '[':
                            parsingList = True
                            listKey = key
                            value = value + char
                            continue

                        if parsingList and char == ']':
                            parsingList = False
                            value = value + char
                            continue

                        if parsingConfigValue and char != ' ' and char != '"':
                            value = value + char
            else:
                value = ""
                for char in line:
                    if char == ',' or char == '\n':
                        if value != '':
                            listCache.append(value)
                        value = ''

                    if char == ']':
                        parsingList = False
                        self.configMap[listKey] = listCache
                        listKey = ''
                        listCache = []
                        break

                    if char != ' ' and char != '"' and char != ',':
                        value = value + char
    
    def __str__(self):
        fileHeader = ''
        fileBody = fileHeader + 'teams = [\n  {\n'
        for key in self.configMap.keys():
            if type(self.configMap[key]) is str:
                fileBody = fileBody + '    ' + key + ' = ' + self.configMap[key] + ';\n'
            elif type(self.configMap[key]) is list:
                table = self.configMap[key]
                fileBody = fileBody + '    ' + key + ' = [\n'
                length = len(table) - 1
                for index, item in enumerate(table):
                    if index < length:
                        fileBody = fileBody + '      ' + item + ',\n'
                    else:
                        fileBody = fileBody + '      ' + item + '\n'
                fileBody = fileBody + '    ' + '];\n'
            else:
                fileBody = fileBody
        fileBody = fileBody + '  ' + '}\n' + '];\n'
        return fileBody

    def __repr__(self):
        return self.__str__()      

    def removePlayer(self, name: str):
        """
        Replaces the transferred robot in a team against an empty robot.
        
        Args:
            name (str): Robot to replace
        """
        players = []
        for player in self.configMap['players']:
            if player == name:
                player = '_'
            players.append(player)
        self.configMap['players'] = players

    def saveTeamsConfig(self):
        """
        Convert Team object to teams.cfg format and saves the result in file.
        """
        teamsFile = open(bhumanBase.teamsFile, 'w')
        teamsFile.write(self.__str__())