#include "Utils/dorsh/tools/ShellTools.h"
#include "Utils/dorsh/tools/StringTools.h"
#include "Utils/dorsh/tools/Platform.h"
#include "Platform/File.h"
#include "Filesystem.h"

std::tuple<QString, QStringList> remoteCommand(const std::string& command, const std::string ip)
{
  std::string ticks = "'";
  return remoteCommandForQProcess(" " + ticks + command + " < /dev/null > /dev/null 2>&1 &" + ticks, ip);
}

std::tuple<QString, QStringList> remoteCommandForQProcess(const std::string& command, const std::string& ip)
{
  return connectCommand(ip + " " + command);
}

std::tuple<QString, QStringList> connectCommand(const std::string& ip)
{
  return {"bash",
      {"-c", QString::fromStdString("cp Keys/id_rsa_nao /tmp/id_rsa_nao && chmod 600 /tmp/id_rsa_nao && ssh -i /tmp/id_rsa_nao -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@" + ip)}};
}

std::tuple<QString, QStringList> scpCommand(const std::string& fromFile, const std::string& fromHost, const std::string& toDir, const std::string& toHost)
{
  std::string from;
  if (fromHost == "")
    from = enquoteString(fromFile);
  else
    from = fromHost + ":" + enquoteString(fromFile);
  std::string to;
  if (toHost == "")
    to = enquoteString(toDir);
  else
    to = toHost + ":" + enquoteString(toDir);

  return {"bash",
      {"-c",
          QString::fromStdString("cp Keys/id_rsa_nao /tmp/id_rsa_nao && chmod 600 /tmp/id_rsa_nao && scp -r -i /tmp/id_rsa_nao -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet "
              + from + " " + to)}};
}

std::tuple<QString, QStringList> scpCommandFromRobot(const std::string& fromDir, const std::string& ip, const std::string& toDir)
{
  return scpCommand(fromDir, "nao@" + ip, toDir, "");
}

std::tuple<QString, QStringList> scpCommandToRobot(const std::string& fromDir, const std::string& ip, const std::string& toDir)
{
  return scpCommand(fromDir, "", toDir, "nao@" + ip);
}
