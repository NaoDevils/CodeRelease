/**
* @file Controller/Views/View3D.cpp
*
* Implementation of class View3D
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author Colin Graf
*/

#include <Platform/OpenGL.h>
#include "View3D.h"
#include "Controller/RoboCupCtrl.h"

#include <QOpenGLWidget>
#include <QMouseEvent>
#include <QSettings>

class View3DWidget : public QOpenGLWidget, public SimRobot::Widget
{
public:
  View3DWidget(View3D& view3D) : view3D(view3D), dragging(false)
  {
    setFocusPolicy(Qt::StrongFocus);

    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(view3D.fullName);
    rotation = settings.value("Rotation").toPointF();
    settings.endGroup();
  }

  virtual ~View3DWidget()
  {
    //saveLayout();
  }

  virtual void saveLayout()
  {
    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(view3D.fullName);
    settings.setValue("Rotation", rotation);
    settings.endGroup();
  }

private:
  void resizeGL(int newWidth, int newHeight)
  {
    width = newWidth;
    height = newHeight;
  }

  void paintGL()
  {
    GLdouble aspect = height ? (GLdouble)width / (GLdouble)height : (GLdouble)width;

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glLineWidth(1.5); // required
    glPointSize(2.5);
    glPolygonMode(GL_FRONT, GL_LINE);
    glPolygonMode(GL_BACK, GL_LINE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_DEPTH_TEST);

    if (view3D.cubeId == 0)
      view3D.cubeId = glGenLists(1);
    if (view3D.colorsId == 0)
      view3D.colorsId = glGenLists(1);

    if (view3D.needsUpdate())
      view3D.updateDisplayLists();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glClearColor(view3D.background.x(), view3D.background.y(), view3D.background.z(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(25, aspect, 1, 100);

    glTranslated(0.0f, 0.0f, -view3D.getViewDistance());
    glRotated(rotation.x(), 1.0f, 0.0f, 0.0f);
    glRotated(rotation.y(), 0.0f, 0.0f, 1.0f);

    glCallList(view3D.cubeId);
    glCallList(view3D.colorsId);

    view3D.lastBackground = view3D.background;
  }

  void mousePressEvent(QMouseEvent* event)
  {
    QWidget::mousePressEvent(event);

    if (event->button() == Qt::LeftButton || event->button() == Qt::MiddleButton)
    {
      dragStart = event->pos();
      dragging = true;
    }
  }

  void mouseReleaseEvent(QMouseEvent* event)
  {
    QWidget::mouseReleaseEvent(event);

    dragging = false;
  }

  void mouseMoveEvent(QMouseEvent* event)
  {
    QWidget::mouseMoveEvent(event);

    if (dragging)
    {
      QPoint diff(event->pos() - dragStart);
      dragStart = event->pos();
      rotation.ry() += diff.x();
      rotation.rx() += diff.y();
      QOpenGLWidget::update();
    }
  }

  void mouseDoubleClickEvent(QMouseEvent* event)
  {
    QWidget::mouseDoubleClickEvent(event);

    rotation = QPointF();
    QOpenGLWidget::update();
  }

  void wheelEvent(QWheelEvent* event)
  {
    if (!event->angleDelta().isNull())
    {
      rotation.ry() += static_cast<float>(event->angleDelta().y()) * 0.2f;
      rotation.rx() += static_cast<float>(event->angleDelta().x()) * 0.2f;
      QOpenGLWidget::update();
    }
    else
      QOpenGLWidget::wheelEvent(event);
  }

  virtual QSize sizeHint() const { return QSize(320, 240); }

  virtual QWidget* getWidget() { return this; }

  virtual void update()
  {
    if (view3D.background != view3D.lastBackground || view3D.needsUpdate())
      QOpenGLWidget::update();
  }

  QPointF rotation;
  int width;
  int height;
  View3D& view3D;
  bool dragging;
  QPoint dragStart;
};

View3D::View3D(const QString& fullName, const Vector3f& background) : background(background), fullName(fullName), icon(":/Icons/tag_green.png") {}

SimRobot::Widget* View3D::createWidget()
{
  return new View3DWidget(*this);
}
