#include <map>
#include <QGuiApplication>
#include <QScreen>
#include <QFormLayout>
#include <QGridLayout>
#include <QLabel>
#include <QScrollArea>

#include "ui/SensorWindow.h"

#include "Platform/File.h"
#include <nlohmann/json.hpp>

SensorWindow::SensorWindow(const nlohmann::json& json, std::string robotName)
{
  setWindowTitle(QString::fromStdString("Sensorview of " + robotName));

  QScrollArea* scrollArea = new QScrollArea();
  scrollArea->setWidgetResizable(true);

  static constexpr int width = 420;
  static constexpr int height = 320;

  scrollArea->setMinimumSize(width, height);

  QWidget* gridWidget = new QWidget(this);

  QGridLayout* gridLayout = new QGridLayout(gridWidget);

  int i = 0;
  const std::function<void(const nlohmann::json&, const std::string&)> displayJson = [&](const nlohmann::json& j, const std::string& prefix)
  {
    if (j.is_object())
    {
      for (const auto& [key, val] : j.items())
        displayJson(val, prefix + (prefix.empty() ? "" : ".") + key);
    }
    else if (j.is_array())
    {
      for (const auto& [key, val] : j.items())
        displayJson(val, prefix + "[" + key + "]");
    }
    else
    {
      QLabel* nameLabel = new QLabel(QString::fromStdString(prefix), gridWidget);
      QLabel* valueLabel = new QLabel(QString::fromStdString(j.dump()), gridWidget);

      gridLayout->addWidget(nameLabel, i, 0, Qt::AlignLeft);
      gridLayout->addWidget(valueLabel, i, 1, Qt::AlignRight);

      ++i;
    }
  };

  displayJson(json, "");


  scrollArea->setWidget(gridWidget);
  gridWidget->setLayout(gridLayout);

  QRect geometry = QGuiApplication::screens().first()->geometry();
  QPoint position((geometry.width() - frameGeometry().width()) / 2 + 200, (geometry.height() - frameGeometry().height()) / 2);

  setCentralWidget(scrollArea);

  setMinimumSize(width, height);

  move(position);
}
