/**
* @file DeleteLogs.cpp
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#include "DeleteLogsCmd.h"
#include "Platform/File.h"
#include "Initializer.h"
#include "cmdlib/Commands.h"
#include "cmdlib/ProcessRunner.h"
#include "tools/Platform.h"
#include "tools/ShellTools.h"


#include <QMessageBox>

DeleteLogsCmd DeleteLogsCmd::theDeleteLogsCmd;

DeleteLogsCmd::DeleteLogsCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string DeleteLogsCmd::getName() const
{
  return "deleteLogs";
}

std::string DeleteLogsCmd::getDescription() const
{
  return "Deletes all *.log files from the robot. Stops and restarts the Framework if necessary";
}

bool DeleteLogsCmd::preExecution(Context& context, const std::vector<std::string>& params)
{
  if (params.size() > 0)
  {
    context.printLine("This command does not have any parameters!");
    return false;
  }

  return true;
}

Task* DeleteLogsCmd::perRobotExecution(Context& context, RobotConfigDorsh& robot)
{
  return new DeleteLogsCmd::DeleteLogsTask(context, &robot);
}

bool DeleteLogsCmd::DeleteLogsTask::execute()
{
  QStringList args;
  args.push_back("--just-delete");
  args.push_back(QString::fromStdString(robot->name));
  args.push_back(QString::fromStdString(robot->getBestIP(context())));

  ProcessRunner r(context(), getCommand(), args);
  r.run();

  if (r.error())
    context().errorLine("Deleting failed!");
  else
    context().printLine("Success! (" + robot->name + ")");

  return true;
}

DeleteLogsCmd::DeleteLogsTask::DeleteLogsTask(Context& context, RobotConfigDorsh* robot) : RobotTask(context, robot) {}

QString DeleteLogsCmd::DeleteLogsTask::getCommand()
{
#ifdef WINDOWS
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + platformDirectory() + "/downloadLogs.cmd");
#else
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + platformDirectory() + "/downloadLogs");
#endif
}
