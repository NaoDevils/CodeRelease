/**
* @file OffscreenRenderer.h
* Declaration of class OffscreenRenderer
* @author Colin Graf
*/

#pragma once

#include <unordered_map>
#include <memory>

class QOpenGLFramebufferObject;
class QOpenGLContext;
class QOffscreenSurface;

/**
* @class OffscreenRenderer
* A hardware accelerated off-screen rendering module that uses the Qt 4 OpenGL library
*/
class OffscreenRenderer
{
public:
  OffscreenRenderer();

  /** Destructor */
  ~OffscreenRenderer();

  /**
  * Prepares the off-screen renderer to render something of a size given with \c ensureSize. This call changes the
  * rendering context to the rendering context of the off-screen renderer.
  * @param width The maximum width.
  * @param height The maximum height.
  */
  void init();

  /**
  * Selects the OpenGL context of the off-screen renderer. Call prepareRendering() before "drawing" the OpenGL image.
  * @param width The width of an image that will be rendered using this off-screen renderer
  * @param height The height of an image that will be rendered using this off-screen renderer
  * @param sampleBuffers Are sample buffers for multi-sampling required?
  */
  void makeCurrent(int width, int height, bool sampleBuffers = true);

  /**
  * Reads an image from current rendering context.
  * @param image The buffer where is image will be saved to.
  * @param width The image width.
  * @param height The image height.
  */
  void finishImageRendering(void* image, int width, int height);

  /**
  * Reads a depth image from current rendering context.
  * @param image The buffer where is image will be saved to.
  * @param width The image width.
  * @param height The image height.
  */
  void finishDepthRendering(void* image, int width, int height);

  /**
  * Accesses the QOpenGLWidget used for rendering. It can be used for creating further QOpenGLWidgets with shared display lists and textures.
  * @return The QOpenGLWidget used for rendering
  */
  QOpenGLContext* getContext() const { return mainGlContext.get(); }

private:
  std::unique_ptr<QOpenGLContext> mainGlContext;
  std::unique_ptr<QOffscreenSurface> mainSurface;

  std::unordered_map<unsigned int, std::unique_ptr<QOpenGLFramebufferObject>> renderBuffers;
  QOpenGLFramebufferObject* currentBuffer = nullptr;

  /**
  * Initializes the currently selected OpenGL context for off-screen rendering
  * @param hasSharedDisplayLists Whether the currently selected context has shared display lists and textures
  */
  void initContext(bool hasSharedDisplayLists);

  std::unique_ptr<QOpenGLFramebufferObject> initPixelBuffer(int width, int height, bool sampleBuffers);
};
