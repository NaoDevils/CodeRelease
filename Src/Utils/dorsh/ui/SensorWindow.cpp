#include <map>
#include <QDesktopWidget>
#include <QFormLayout>
#include <QGridLayout>
#include <QLabel>
#include <QScrollArea>

#include "Utils/dorsh/ui/SensorWindow.h"

#include "Platform/File.h"

SensorWindow::SensorWindow(std::map<std::string, std::string> map, std::string robotName)
{
  std::string buf("Sensorview of ");
  buf.append(robotName);
  setWindowTitle(buf.c_str());

  QScrollArea* scrollArea = new QScrollArea();
  scrollArea->setWidgetResizable(true);

  int width = 420;
  int height = 320;

  scrollArea->setMinimumSize(width, height);
  scrollArea->setMaximumSize(width, height);

  QWidget* gridWidget = new QWidget();

  QGridLayout* gridLayout = new QGridLayout(gridWidget);
  auto it = map.cbegin(), end = map.cend();
  for (int i = 0, j = 0; it != end; ++it, i ++)
  {
    QLabel* nameLabel = new QLabel(it->first.c_str(), gridWidget);
    QLabel* valueLabel = new QLabel(it->second.c_str(), gridWidget);

    if (i % 2 == 0)
    {
      nameLabel->setStyleSheet("QLabel {color : Black; }");
      valueLabel->setStyleSheet("QLabel {color : Black; }");
    }
    if (i % 2 == 1)
    {
      nameLabel->setStyleSheet("QLabel {color : Navy; }");
      valueLabel->setStyleSheet("QLabel {color : Navy; }");
    }

    gridLayout->addWidget(nameLabel, i, j, Qt::AlignLeft);
    gridLayout->addWidget(valueLabel, i, j + 1, Qt::AlignRight);
    
    if (i >= 200)
    {
      i = -1;
      j += 2;
    }
  }

  scrollArea->setWidget(gridWidget);
  gridWidget->setLayout(gridLayout);

  QDesktopWidget desktop;
  QPoint position((desktop.screenGeometry().width() - frameGeometry().width()) / 2 + 200, (desktop.screenGeometry().height() - frameGeometry().height()) / 2);

  QWidget::setLayout(new QVBoxLayout);
  QWidget::layout()->addWidget(scrollArea);

  QWidget::setMinimumSize(width, height);
  QWidget::setMaximumSize(width, height);

  QWidget::move(position);
}
