#include <QString>
#include "Utils/dorsh/cmdlib/AbstractConsole.h"

AbstractConsole::AbstractConsole(QObject* parent) : QObject(parent), IConsole()
{
  qRegisterMetaType<ConsolePrintTarget>();
}

void AbstractConsole::print(const std::string& msg)
{
  emit sPrint(CPT_PRINT, QString::fromStdString(msg));
}

void AbstractConsole::printLine(const std::string& msg)
{
  emit sPrint(CPT_PRINT_LINE, QString::fromStdString(msg));
}

void AbstractConsole::error(const std::string& msg)
{
  emit sPrint(CPT_ERROR, QString::fromStdString(msg));
}

void AbstractConsole::errorLine(const std::string& msg)
{
  emit sPrint(CPT_ERROR_LINE, QString::fromStdString(msg));
}
