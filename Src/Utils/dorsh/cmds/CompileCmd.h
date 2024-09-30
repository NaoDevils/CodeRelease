#pragma once

#include <QObject>
#include "cmdlib/CommandAdapter.h"
#include "cmdlib/Commands.h"
#include "cmdlib/Context.h"
#include "cmdlib/ProcessRunner.h"

class CompileCmd : public CommandAdapter
{
  class CompileTask : public Task
  {
    ProcessRunner r;
    const std::string label;

  public:
    CompileTask(Context& context, const std::string& label, const QString& command, const QStringList& args);
    bool execute();
    void cancel();
    void setContext(Context* context);
    std::string getLabel();
  };
  std::string vsPath = "";

  CompileCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual std::vector<std::string> complete(const std::string& cmdLine) const;
  virtual bool execute(Context& context, const std::vector<std::string>& params);
  QString getCommand(const QString& project);
  QStringList getParams(const QString& config, const QString& project);

public:
  static CompileCmd theCompileCmd;

private:
  QString wslpath(const QString& path);
};
