#pragma once

#include <QString>
#include <QStringList>
#include <tuple>

std::tuple<QString, QStringList> remoteCommand(const std::string& command, const std::string ip);
std::tuple<QString, QStringList> remoteCommandForQProcess(const std::string& command, const std::string& ip);
std::tuple<QString, QStringList> connectCommand(const std::string& ip);

std::tuple<QString, QStringList> scpCommand(const std::string& fromDir, const std::string& fromHost, const std::string& toDir, const std::string& toHost);
std::tuple<QString, QStringList> scpCommandFromRobot(const std::string& fromDir, const std::string& ip, const std::string& toDir);
std::tuple<QString, QStringList> scpCommandToRobot(const std::string& fromDir, const std::string& ip, const std::string& toDir);
