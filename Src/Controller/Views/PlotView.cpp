/**
* @file Controller/Views/PlotView.cpp
*
* Implementation of class PlotView
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author Colin Graf
*/

#include <QMouseEvent>
#include <QMenu>
#include <QSettings>
#include <QFileDialog>
#include <QTextStream>

#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "PlotView.h"
#include <algorithm>

PlotWidget::PlotWidget(PlotView& plotView, PlotWidget*& plotWidget)
    : plotView(plotView), plotWidget(plotWidget), lastTimeStamp(0), blackPen(QColor(0x00, 0x00, 0x00)), grayPen(QColor(0xbb, 0xbb, 0xbb)), drawUnits(true), drawLegend(true),
      continousMinMax(false), antialiasing(false)
{
  setFocusPolicy(Qt::StrongFocus);
  setBackgroundRole(QPalette::Base);
  setAutoFillBackground(true);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(plotView.fullName);
  drawUnits = settings.value("DrawUnits", true).toBool();
  drawLegend = settings.value("DrawLegend", true).toBool();
  continousMinMax = settings.value("ContinousMinMax", false).toBool();
  antialiasing = settings.value("Antialiasing", false).toBool();
  settings.endGroup();
  blackPen.setWidth(0);
  grayPen.setWidth(0);
}

PlotWidget::~PlotWidget()
{
  //saveLayout();

  plotWidget = 0;
}

void PlotWidget::saveLayout()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(plotView.fullName);
  settings.setValue("DrawUnits", drawUnits);
  settings.setValue("DrawLegend", drawLegend);
  settings.setValue("ContinousMinMax", continousMinMax);
  settings.setValue("Antialiasing", antialiasing);
  settings.endGroup();
}

bool PlotWidget::needsRepaint() const
{
  SYNC_WITH(plotView.console);
  for (std::list<RobotConsole::Layer>::const_iterator i = plotView.console.plotViews[plotView.name].begin(), end = plotView.console.plotViews[plotView.name].end(); i != end; ++i)
    if (plotView.console.plots[i->layer].timeStamp > lastTimeStamp)
      return true;
  return false;
}

void PlotWidget::paintEvent(QPaintEvent* event)
{
  painter.begin(this);
  if (antialiasing)
    painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);
  paint(painter);
  painter.end();
}

void PlotWidget::paint(QPainter& painter)
{
  // calculate margin sizes
  float plotSizeF = (float)plotView.plotSize;
  const float space = 3;
  float leftMargin;
  float bottomMargin;
  float topMargin;
  float rightMargin;
  float textHeight;
  const QFontMetrics& fontMetrics(painter.fontMetrics());
  {
    char buf[32];
    sprintf(buf, "%g", (plotView.maxValue - plotView.minValue) < 8. ? (plotView.minValue / 4.) : plotView.minValue);
    const QSize& bufSize(fontMetrics.size(Qt::TextSingleLine, buf));
    textHeight = bufSize.height();
    leftMargin = bufSize.width();
    bottomMargin = textHeight + space * 2;
    topMargin = space + textHeight / 2.f;
    sprintf(buf, "%g", (plotView.maxValue - plotView.minValue) < 8. ? (plotView.maxValue / 4.) : plotView.maxValue);
    const QSize& bufSize2(fontMetrics.size(Qt::TextSingleLine, buf));
    if (bufSize2.width() > leftMargin)
      leftMargin = bufSize2.width();
    rightMargin = fontMetrics.size(Qt::TextSingleLine, "0").width() / 2.f + space;
    leftMargin += space * 2;
    if (drawUnits)
    {
      if (!plotView.yUnit.empty())
        topMargin += textHeight + space;
      if (!plotView.xUnit.empty())
        rightMargin += fontMetrics.size(Qt::TextSingleLine, plotView.xUnit.c_str()).width() + space;
    }
  }

  // calculate size of the plot area
  const QRect& windowRect(painter.window());
  QRect plotRect(QPoint((int)(windowRect.x() + leftMargin), (int)(windowRect.y() + topMargin)), QPoint((int)(windowRect.right() - rightMargin), (int)(windowRect.bottom() - bottomMargin)));
  if (!plotRect.isValid())
    return; // window too small

  // calculate step sizes to regrade the axes
  float stepY = std::pow(10.f, std::ceil(std::log10(plotView.valueLength * 20.f / plotRect.height())));
  float stepX = std::pow(10.f, std::ceil(std::log10(plotSizeF * 25.f / plotRect.width())));
  if (stepY * plotRect.height() / plotView.valueLength >= 40.f)
    stepY /= 2.f;
  if (stepY * plotRect.height() / plotView.valueLength >= 40.f)
    stepY /= 2.f;
  if (stepY > std::max<>(plotView.maxValue, -plotView.minValue))
    stepY = std::max<>(plotView.maxValue, -plotView.minValue);
  if (plotRect.width() * stepX / plotSizeF >= 50.f)
    stepX /= 2.f;
  if (plotRect.width() * stepX / plotSizeF >= 50.f)
    stepX /= 2.f;
  if (stepX > plotSizeF)
    stepX = plotSizeF;

  if ((plotView.maxValue - plotView.minValue) < 8.f)
  {
    char buf[32];
    sprintf(buf, "%g", stepY < std::abs(plotView.maxValue) ? (plotView.maxValue - stepY) : plotView.maxValue);
    int width = fontMetrics.size(Qt::TextSingleLine, buf).width();
    sprintf(buf, "%g", stepY < std::abs(plotView.minValue) ? (plotView.minValue + stepY) : plotView.minValue);
    int width2 = fontMetrics.size(Qt::TextSingleLine, buf).width();
    if (width2 > width)
      width = width2;
    width += static_cast<int>(space * 2);
    if (width < leftMargin)
    {
      plotRect.setLeft((int)(plotRect.left() - (leftMargin - width)));
      leftMargin = width;
    }
  }

  // draw axis lables
  {
    char buf[32];
    painter.setPen(blackPen);

    // y
    {
      QRectF rect(0, 0, leftMargin - space, textHeight);
      sprintf(buf, "%g", plotView.maxValue);
      rect.moveTop((plotView.valueLength - (plotView.maxValue - plotView.minValue)) * plotRect.height() / plotView.valueLength + topMargin - textHeight / 2.);
      painter.drawText(rect, Qt::AlignRight, tr(buf));
      sprintf(buf, "%g", plotView.minValue);
      rect.moveTop((plotView.valueLength - (plotView.minValue - plotView.minValue)) * plotRect.height() / plotView.valueLength + topMargin - textHeight / 2.);
      painter.drawText(rect, Qt::AlignRight, tr(buf));
      rect.moveTop((plotView.valueLength - (0. - plotView.minValue)) * plotRect.height() / plotView.valueLength + topMargin - textHeight / 2.);
      painter.drawText(rect, Qt::AlignRight, QString("0"));
      const float& minOffset(plotView.valueLength * 20.f / plotRect.height());
      for (float pos = stepY; pos <= plotView.maxValue - minOffset; pos += stepY)
      {
        sprintf(buf, "%g", pos);
        rect.moveTop((plotView.valueLength - (pos - plotView.minValue)) * plotRect.height() / plotView.valueLength + topMargin - textHeight / 2.);
        painter.drawText(rect, Qt::AlignRight, tr(buf));
      }
      for (float pos = -stepY; pos >= plotView.minValue + minOffset; pos -= stepY)
      {
        sprintf(buf, "%g", pos);
        rect.moveTop(((plotView.maxValue - plotView.minValue) - (pos - plotView.minValue)) * plotRect.height() / plotView.valueLength + topMargin - textHeight / 2.);
        painter.drawText(rect, Qt::AlignRight, tr(buf));
      }
    }

    // x
    {
      QRectF rect(0, plotRect.bottom() + space, leftMargin - space, textHeight);
      sprintf(buf, "%.2f", plotSizeF * plotView.xScale);
      rect.setWidth(fontMetrics.size(Qt::TextSingleLine, buf).width());
      sprintf(buf, "%g", plotSizeF * plotView.xScale);
      rect.moveLeft((plotSizeF - plotSizeF) * plotRect.width() / plotSizeF + leftMargin - rect.width() / 2.);
      painter.drawText(rect, Qt::AlignCenter, tr(buf));
      rect.moveLeft((plotSizeF - 0.) * plotRect.width() / plotSizeF + leftMargin - rect.width() / 2.);
      painter.drawText(rect, Qt::AlignCenter, QString("0"));
      const float& minOffset(plotSizeF * 25.f / plotRect.width());
      for (float pos = stepX; pos <= plotSizeF - minOffset; pos += stepX)
      {
        sprintf(buf, "%g", pos * plotView.xScale);
        rect.moveLeft((plotSizeF - pos) * plotRect.width() / plotSizeF + leftMargin - rect.width() / 2.);
        painter.drawText(rect, Qt::AlignCenter, tr(buf));
      }
    }
  }

  // draw units
  if (drawUnits)
  {
    if (!plotView.yUnit.empty())
    {
      QRect rect((int)space, (int)space, (int)(windowRect.width() - space), (int)textHeight);
      painter.drawText(rect, Qt::AlignLeft, tr(plotView.yUnit.c_str()));
    }
    if (!plotView.xUnit.empty())
    {
      QRect rect(0, (int)(plotRect.bottom() + space), (int)(windowRect.width() - space), (int)textHeight);
      painter.drawText(rect, Qt::AlignRight, tr(plotView.xUnit.c_str()));
    }
  }

  // setup plot-paint-transformation
  {
    QTransform transform;
    transform.translate(plotRect.right(), plotRect.top() + float(plotRect.height()) * plotView.maxValue / plotView.valueLength);
    transform.scale(-float(plotRect.width()) / plotSizeF, -float(plotRect.height()) / plotView.valueLength);
    painter.setTransform(transform);
  }

  // draw axes and regrade-lines
  painter.setPen(grayPen);
  float twoPxX = plotSizeF * 2.f / plotRect.width();
  float twoPxY = plotView.valueLength * 2.f / plotRect.height();
  for (float pos = stepY; pos < plotView.maxValue; pos += stepY)
    painter.drawLine(QPointF(0., pos), QPointF(plotSizeF + twoPxX, pos));
  for (float pos = -stepY; pos > plotView.minValue; pos -= stepY)
    painter.drawLine(QPointF(0., pos), QPointF(plotSizeF + twoPxX, pos));
  for (float pos = stepX; pos < plotSizeF; pos += stepX)
    painter.drawLine(QPointF(pos, plotView.minValue - twoPxY), QPointF(pos, plotView.maxValue));
  painter.setPen(blackPen);
  painter.drawLine(QPointF(0., 0.), QPointF(plotSizeF + twoPxX, 0.));
  painter.drawLine(QPointF(0., plotView.minValue - twoPxY), QPointF(0., plotView.maxValue));
  painter.drawLine(QPointF(plotSizeF, plotView.minValue - twoPxY), QPointF(plotSizeF, plotView.maxValue));
  if (plotView.minValue != 0.)
    painter.drawLine(QPointF(0., plotView.minValue), QPointF(plotSizeF + twoPxX, plotView.minValue));
  if (plotView.maxValue != 0.)
    painter.drawLine(QPointF(0., plotView.maxValue), QPointF(plotSizeF + twoPxX, plotView.maxValue));

  {
    SYNC_WITH(plotView.console);

    // draw plots
    if (antialiasing)
      painter.setRenderHints(QPainter::Antialiasing);
    int legendWidth = 0;
    const std::list<RobotConsole::Layer>& plotList(plotView.console.plotViews[plotView.name]);
    bool started = false;
    for (std::list<RobotConsole::Layer>::const_iterator i = plotList.begin(), end = plotList.end(); i != end; ++i)
    {
      const std::list<float>& list = plotView.console.plots[i->layer].points;
      int numOfPoints = std::min((int)list.size(), (int)plotView.plotSize);
      if (numOfPoints > 1)
      {
        std::list<float>::const_iterator k = list.end();
        for (int j = plotView.plotSize - 1; j >= (int)plotView.plotSize - numOfPoints; --j)
        {
          const float& value(*(k--));
          plotView.points[j].ry() = value;
          if (started && continousMinMax)
          {
            if (value < plotView.minValue)
              plotView.minValue = value;
            else if (value > plotView.maxValue)
              plotView.maxValue = value;
          }
          else if (continousMinMax)
          {
            plotView.minValue = value;
            plotView.maxValue = value + 0.01f;
            started = true;
          }
        }

        const ColorRGBA& color(i->color);
        QPen pen(QColor(color.r, color.g, color.b));
        pen.setWidth(0);
        painter.setPen(pen);
        painter.drawPolyline(plotView.points + (plotView.plotSize - numOfPoints), numOfPoints);
      }
      unsigned int timeStamp = plotView.console.plots[i->layer].timeStamp;
      if (timeStamp > lastTimeStamp)
        lastTimeStamp = timeStamp;
      if (drawLegend)
      {
        int width = fontMetrics.size(Qt::TextSingleLine, i->description.c_str()).width();
        if (width > legendWidth)
          legendWidth = width;
      }
    }

    if (started && continousMinMax)
    {
      float diff = plotView.maxValue - plotView.minValue;
      float logTen = std::log10(diff * 2);
      int precision = int(std::ceil(logTen)) - 1;
      float rounder = std::pow(10.f, (float)precision);
      plotView.maxValue = std::ceil(plotView.maxValue / rounder) * rounder;
      plotView.minValue = std::floor(plotView.minValue / rounder) * rounder;
      plotView.valueLength = plotView.maxValue - plotView.minValue;
    }

    if (antialiasing)
      painter.setRenderHints(QPainter::Antialiasing, false);

    // draw legend
    if (drawLegend && plotList.size() > 0)
    {
      QRect legendRect((int)(plotRect.left() + space), (int)(plotRect.top() + space), (int)(legendWidth + space * 3 + 10), (int)(space + plotList.size() * (textHeight + space)));
      QRect rect((int)(legendRect.left() + space + 10 + space), (int)(legendRect.top() + space), (int)(legendRect.width() - (space + 10 + space)), (int)textHeight);
      QLine line((int)(legendRect.left() + space), (int)(legendRect.top() + space + textHeight / 2 + 1), (int)(legendRect.left() + space + 10), (int)(legendRect.top() + space + textHeight / 2 + 1));
      painter.setTransform(QTransform());
      painter.setPen(blackPen);
      painter.setBrush(QBrush(QColor(0xff, 0xff, 0xff, 0x99)));
      painter.drawRect(legendRect);
      for (std::list<RobotConsole::Layer>::const_iterator i = plotList.begin(), end = plotList.end(); i != end; ++i)
      {
        painter.setPen(blackPen);
        painter.drawText(rect, Qt::AlignLeft, tr(i->description.c_str()));
        const ColorRGBA& color(i->color);
        painter.setPen(QColor(color.r, color.g, color.b));
        painter.drawLine(line);
        rect.moveTop((int)(rect.top() + textHeight + space));
        line.translate(0, (int)(textHeight + space));
      }
    }
  }
}

void PlotWidget::determineMinMaxValue()
{
  bool started = false;
  {
    SYNC_WITH(plotView.console);
    const std::list<RobotConsole::Layer>& plotList(plotView.console.plotViews[plotView.name]);
    for (std::list<RobotConsole::Layer>::const_iterator i = plotList.begin(), end = plotList.end(); i != end; ++i)
    {
      const std::list<float>& list = plotView.console.plots[i->layer].points;
      int numOfPoints = std::min((int)list.size(), (int)plotView.plotSize);
      if (numOfPoints > 1)
      {
        std::list<float>::const_iterator k = list.end();
        for (int j = plotView.plotSize - 1; j >= (int)plotView.plotSize - numOfPoints; --j)
        {
          const float& value(*(k--));
          if (started)
          {
            if (value < plotView.minValue)
              plotView.minValue = value;
            else if (value > plotView.maxValue)
              plotView.maxValue = value;
          }
          else
          {
            plotView.minValue = value;
            plotView.maxValue = plotView.minValue + 0.00001f;
            started = true;
          }
        }
      }
    }
  }

  if (started)
  {
    int precision = int(std::ceil(std::log10(plotView.maxValue - plotView.minValue))) - 1;
    float rounder = std::pow(10.f, (float)precision);
    plotView.minValue = std::floor(plotView.minValue / rounder) * rounder;
    plotView.maxValue = std::ceil(plotView.maxValue / rounder) * rounder;
    plotView.valueLength = plotView.maxValue - plotView.minValue;
  }

  QWidget::update();
}

void PlotWidget::toggleDrawUnits()
{
  drawUnits = !drawUnits;
  QWidget::update();
}

void PlotWidget::toggleDrawLegend()
{
  drawLegend = !drawLegend;
  QWidget::update();
}

void PlotWidget::toggleAntialiasing()
{
  antialiasing = !antialiasing;
  QWidget::update();
}

void PlotWidget::toggleContinousMinMax()
{
  continousMinMax = !continousMinMax;
  QWidget::update();
}

void PlotWidget::exportAsGnuplot()
{
  // ask for destination files
  QSettings& settings = RoboCupCtrl::application->getSettings();
  QString fileName = QFileDialog::getSaveFileName(this, tr("Export as Gnuplot"), settings.value("ExportDirectory", "").toString(), tr("Gnuplot (*.plt)"));
  if (fileName.isEmpty())
    return;
  settings.setValue("ExportDirectory", QFileInfo(fileName).dir().path());

  // prepare plot data
  SYNC_WITH(plotView.console);
  QVector<QVector<float>> data;
  int numOfPoints = plotView.plotSize;
  int numOfPlots = 0;
  {
    const std::list<RobotConsole::Layer>& plotList(plotView.console.plotViews[plotView.name]);
    numOfPlots = (int)plotList.size();
    for (std::list<RobotConsole::Layer>::const_iterator i = plotList.begin(), end = plotList.end(); i != end; ++i)
    {
      const std::list<float>& list = plotView.console.plots[i->layer].points;
      int curNumOfPoints = std::min((int)list.size(), (int)plotView.plotSize);
      if (curNumOfPoints < numOfPoints)
        numOfPoints = curNumOfPoints;
    }
  }
  if (!numOfPoints || !numOfPlots)
    return;
  data.resize(numOfPoints);
  for (int i = 0; i < numOfPoints; ++i)
    data[i].resize(numOfPlots);

  {
    const std::list<RobotConsole::Layer>& plotList(plotView.console.plotViews[plotView.name]);
    int currentPlot = 0;
    for (std::list<RobotConsole::Layer>::const_iterator i = plotList.begin(), end = plotList.end(); i != end; ++i, ++currentPlot)
    {
      const std::list<float>& list = plotView.console.plots[i->layer].points;
      std::list<float>::const_reverse_iterator k = list.rbegin();
      for (int j = numOfPoints - 1; j >= 0; --j)
        data[j][currentPlot] = *(k++);
    }
  }

  // open output steams
  QFileInfo fileInfo(fileName);
  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    return;
  QTextStream out(&file);
  QFile dataFile(fileInfo.dir().path() + "/" + fileInfo.baseName() + ".dat");
  if (!dataFile.open(QIODevice::WriteOnly | QIODevice::Text))
    return;
  QTextStream dataOut(&dataFile);

  // create .plt file
  out << "reset\n";
  out << "set title \"" << QString(plotView.name.c_str()) << "\"\n";
  if (!plotView.xUnit.empty())
    out << "set xlabel \"[" << QString(plotView.xUnit.c_str()) << "]\"\n";
  else
    out << "#set xlabel \"x\"\n";
  if (!plotView.yUnit.empty())
    out << "set ylabel \"[" << QString(plotView.yUnit.c_str()) << "]\"\n";
  else
    out << "#set ylabel \"y\"\n";
  out << "set xrange [" << float(plotView.plotSize) * plotView.xScale << ":" << 0 << "]\n";
  out << "set yrange [" << plotView.minValue << ":" << plotView.maxValue << "]\n";
  out << "set terminal postscript eps enhanced color\n";
  out << "set output \"" << fileInfo.baseName() << ".eps\"\n";
  const std::list<RobotConsole::Layer>& plotList(plotView.console.plotViews[plotView.name]);
  int currentPlot = 0;
  for (std::list<RobotConsole::Layer>::const_iterator i = plotList.begin(), end = plotList.end(); i != end; ++i, ++currentPlot)
  {
    if (currentPlot == 0)
      out << "plot ";
    else
      out << ", ";
    out << "\"" << fileInfo.baseName() << ".dat\" using 1:" << currentPlot + 2 << " title \"" << i->description.c_str() << "\"";
    char rgbcolor[10];
    sprintf(rgbcolor, "%06X", i->color.r << 16 | i->color.g << 8 | i->color.b);
    out << " with lines linetype 1 linecolor rgbcolor \"#" << rgbcolor << "\"";
  }
  out << "\n";

  // create .dat file
  for (int i = 0, j = plotView.plotSize - numOfPoints; j < int(plotView.plotSize); ++j, ++i)
  {
    dataOut << float(plotView.plotSize - 1 - j) * plotView.xScale;
    for (int k = 0; k < numOfPlots; ++k)
      dataOut << " " << data[i][k];
    dataOut << "\n";
  }
}

QMenu* PlotWidget::createUserMenu() const
{
  QMenu* menu = new QMenu(tr("&Plot"));

  QAction* action = menu->addAction(tr("Export as Gnuplot..."));
  connect(action, SIGNAL(triggered()), this, SLOT(exportAsGnuplot()));

  menu->addSeparator();
  action = menu->addAction(tr("Show Units"));
  action->setCheckable(true);
  action->setChecked(drawUnits);
  connect(action, SIGNAL(triggered()), this, SLOT(toggleDrawUnits()));
  action = menu->addAction(tr("Show Key"));
  action->setCheckable(true);
  action->setChecked(drawLegend);
  connect(action, SIGNAL(triggered()), this, SLOT(toggleDrawLegend()));
  action = menu->addAction(tr("Anti-aliased"));
  action->setCheckable(true);
  action->setChecked(antialiasing);
  connect(action, SIGNAL(triggered()), this, SLOT(toggleAntialiasing()));

  menu->addSeparator();
  action = menu->addAction(tr("Auto-min-max"));
  connect(action, SIGNAL(triggered()), this, SLOT(determineMinMaxValue()));

  action = menu->addAction(tr("Continous Auto-min-max"));
  action->setCheckable(true);
  action->setChecked(continousMinMax);
  connect(action, SIGNAL(triggered()), this, SLOT(toggleContinousMinMax()));


  return menu;
}

void PlotWidget::update()
{
  if (needsRepaint())
    QWidget::update();
}

PlotView::PlotView(
    const QString& fullName, RobotConsole& console, const std::string& name, unsigned int plotSize, float minValue, float maxValue, const std::string& yUnit, const std::string& xUnit, float xScale)
    : fullName(fullName), icon(":/Icons/tag_green.png"), console(console), plotWidget(0), name(name), plotSize(plotSize), minValue(minValue), maxValue(maxValue),
      valueLength(maxValue - minValue), yUnit(yUnit), xUnit(xUnit), xScale(xScale)
{
  points = new QPointF[plotSize];
  for (unsigned int i = 0; i < plotSize; ++i)
    points[i].rx() = plotSize - 1 - i;
}

PlotView::~PlotView()
{
  delete[] points;
}

void PlotView::setParameters(unsigned int plotSize, float minValue, float maxValue, const std::string& yUnit, const std::string& xUnit, float xScale)
{
  delete[] points;
  points = new QPointF[plotSize];
  for (unsigned int i = 0; i < plotSize; ++i)
    points[i].rx() = plotSize - 1 - i;

  this->plotSize = plotSize;
  this->minValue = minValue;
  this->maxValue = maxValue;
  this->valueLength = maxValue - minValue;
  this->yUnit = yUnit;
  this->xUnit = xUnit;
  this->xScale = xScale;
  if (plotWidget)
    ((QWidget*)plotWidget)->update();
}

SimRobot::Widget* PlotView::createWidget()
{
  ASSERT(!plotWidget);
  plotWidget = new PlotWidget(*this, plotWidget);
  return plotWidget;
}
