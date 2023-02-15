/**
 * @file OffscreenRenderer.cpp
 * An implementation of a hardware accelerated off-screen rendering module.
 * @author Colin Graf
 */

#include "Platform/OpenGL.h"
#include <QOpenGLFramebufferObject>

// Hide Qt warning about GLEW on Linux
#undef __GLEW_H__
#include <QOpenGLContext>
#define __GLEW_H__

#include <QOffscreenSurface>

#include "Platform/OffscreenRenderer.h"
#include "Platform/Assert.h"
#include "Simulation/Simulation.h"
#include "Simulation/Scene.h"
#include "SimObjectWidget.h"

OffscreenRenderer::OffscreenRenderer() = default;
OffscreenRenderer::~OffscreenRenderer() = default;

void OffscreenRenderer::makeCurrent(int width, int height, bool sampleBuffers)
{
  ASSERT(mainGlContext);
  ASSERT(currentBuffer == nullptr);

  mainGlContext->makeCurrent(mainSurface.get());

  // Considering weak graphics cards glClear is faster when the color and depth buffers are not greater then they have to be.
  // So we create an individual buffer for each size in demand.
  const auto it = renderBuffers.find(width << 16 | height << 1 | (sampleBuffers ? 1 : 0));
  if (it == renderBuffers.end())
  {
    auto& buffer = renderBuffers[width << 16 | height << 1 | (sampleBuffers ? 1 : 0)];

    VERIFY(buffer = initPixelBuffer(width, height, sampleBuffers));
    currentBuffer = buffer.get();
  }
  else
  {
    const auto& buffer = it->second;

    VERIFY(buffer->bind());
    currentBuffer = buffer.get();
  }
}

std::unique_ptr<QOpenGLFramebufferObject> OffscreenRenderer::initPixelBuffer(int width, int height, bool sampleBuffers)
{
  QOpenGLFramebufferObjectFormat format;
  format.setSamples(sampleBuffers ? 8 : 0);
  format.setAttachment(QOpenGLFramebufferObject::Attachment::Depth);
  std::unique_ptr<QOpenGLFramebufferObject> buf = std::make_unique<QOpenGLFramebufferObject>(QSize(width, height), format);
  if (!buf->isValid())
  {
    return nullptr;
  }

  VERIFY(buf->bind());
  initContext(true);
  return buf;
}


void OffscreenRenderer::initContext(bool hasSharedDisplayLists)
{
#ifndef MACOS
  glewInit();
#endif

  Simulation::simulation->scene->createGraphics(hasSharedDisplayLists);
}

void OffscreenRenderer::init()
{
  ASSERT(!mainGlContext);

  mainGlContext = std::make_unique<QOpenGLContext>();
  mainGlContext->setShareContext(QOpenGLContext::globalShareContext());
  mainGlContext->create();
  if (!mainGlContext->isValid())
  {
    qWarning() << "Cannot initialize OpenGL context. 3D rendering not available!";
    mainGlContext.reset();
    return;
  }

  mainSurface = std::make_unique<QOffscreenSurface>();
  mainSurface->create();
  VERIFY(mainSurface->isValid());

  mainGlContext->makeCurrent(mainSurface.get());

  /* debugging: (also enable debug context in Main.cpp)
  QOpenGLDebugLogger* logger = new QOpenGLDebugLogger;
  VERIFY(logger->initialize());
  QObject::connect(logger, &QOpenGLDebugLogger::messageLogged, [&](const auto& msg)
    {
      qDebug() << msg.message();
    });
  logger->startLogging(QOpenGLDebugLogger::SynchronousLogging);
  */

  initContext(false);
}

void OffscreenRenderer::finishImageRendering(void* image, int w, int h)
{
  ASSERT(currentBuffer);

  if (w * h > 0)
  {
    const auto getImage = [&]
    {
      const int lineSize = w * 3;
      glPixelStorei(GL_PACK_ALIGNMENT, lineSize & (8 - 1) ? (lineSize & (4 - 1) ? 1 : 4) : 8);
      glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, image);
    };

    if (currentBuffer->format().samples() > 0)
    {
      QRect rect(QPoint(0, 0), currentBuffer->size());
      QOpenGLFramebufferObject temp(currentBuffer->size());
      QOpenGLFramebufferObject::blitFramebuffer(&temp, rect, currentBuffer, rect, GL_COLOR_BUFFER_BIT, GL_NEAREST, 0, 0, QOpenGLFramebufferObject::DontRestoreFramebufferBinding);
      currentBuffer->release();

      temp.bind();
      getImage();
      temp.release();
    }
    else
    {
      getImage();
      currentBuffer->release();
    }

    // native Qt functions seem to copy the image 4 times
    //QImage qimage = currentBuffer->toImage(false);
    //qimage.convertTo(QImage::Format_RGB888);
    //memcpy(image, qimage.bits(), qimage.width() * qimage.height() * qimage.depth() / 8);
  }
  else
  {
    currentBuffer->release();
  }

  currentBuffer = nullptr;
}

void OffscreenRenderer::finishDepthRendering(void* image, int w, int h)
{
  ASSERT(currentBuffer);
  currentBuffer->release();

  glPixelStorei(GL_PACK_ALIGNMENT, w * 4 & (8 - 1) ? 4 : 8);
  glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, image);

  currentBuffer = nullptr;
}
