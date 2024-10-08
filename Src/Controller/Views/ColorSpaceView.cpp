/**
* @file Controller/Views/ColorSpaceView.cpp
*
* Implementation of class ColorSpaceView
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "Controller/RobotConsole.h"
#include "ColorSpaceView.h"
#include "Platform/Thread.h"
#include "Controller/Visualization/OpenGLMethods.h"
#ifdef MACOS
#include <OpenGL/OpenGL.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#endif


ColorSpaceView::ColorSpaceView(const QString& fullName, RobotConsole& c, const std::string& n, ColorModel cm, int ch, const Vector3f& b, bool upperCam)
    : View3D(fullName, b), console(c), name(n), colorModel(cm), channel(ch), lastTimeStamp(0), upperCam(upperCam)
{
}

void ColorSpaceView::updateDisplayLists()
{
  SYNC_WITH(console);
  Image *image = 0, *raw = 0;

  RobotConsole::Images& currentImages = console.camImages;
  RobotConsole::Images::const_iterator i = currentImages.find(name);
  std::string imageString = upperCam ? "raw imageUpper" : "raw image";

  if (i != currentImages.end())
    image = i->second.get();
  i = currentImages.find(imageString);
  if (i != currentImages.end())
    raw = i->second.get();
  if (image && (channel < 3 || raw))
  {
    if (!channel)
      OpenGLMethods::paintCubeToOpenGLList(256,
          256,
          256,
          cubeId,
          true,
          127, //scale
          -127,
          -127,
          -127, // offsets
          int(background.x() * 255) ^ 0xc0,
          int(background.y() * 255) ^ 0xc0,
          int(background.z() * 255) ^ 0xc0);
    else
      OpenGLMethods::paintCubeToOpenGLList(image->width,
          image->height,
          128,
          cubeId,
          true,
          127, //scale
          -image->width / 2,
          -image->height / 2,
          -65, // offsets
          int(background.x() * 255) ^ 0xc0,
          int(background.y() * 255) ^ 0xc0,
          int(background.z() * 255) ^ 0xc0);

    OpenGLMethods::paintImagePixelsToOpenGLList(*image, colorModel, channel - 1, false, colorsId);
    lastTimeStamp = image->timeStamp;
  }
  else
  {
    glNewList(cubeId, GL_COMPILE_AND_EXECUTE);
    glEndList();
    glNewList(colorsId, GL_COMPILE_AND_EXECUTE);
    glEndList();
    lastTimeStamp = 0;
  }
}

bool ColorSpaceView::needsUpdate() const
{
  SYNC_WITH(console);
  Image* image = 0;
  RobotConsole::Images& currentImages = console.camImages;
  RobotConsole::Images::const_iterator i = currentImages.find(name);
  if (i != currentImages.end())
    image = i->second.get();
  return ((image && image->timeStamp != lastTimeStamp) || (!image && lastTimeStamp));
}
