/**
 * @file Platform/Linux/NaoCameraV6.cpp
 * Interface to a camera of the NAO.
 * @author Colin Graf
 * @author Thomas Röfer
 */

#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cerrno>
#include <poll.h>
#include <linux/videodev2.h>

#include <thread>
#include <chrono>

#include "NaoCameraV6.h"
#include "BHAssert.h"
#include "SystemCall.h"
#include "Tools/Debugging/Debugging.h"

#include "linux/uvcvideo.h"
#include "linux/usb/video.h"

NaoCameraV6::NaoCameraV6(const char* device, bool upper, int width, int height, bool flip) :
  upper(upper),
  WIDTH(width * 2),
  HEIGHT(height * 2)
#ifndef NDEBUG
  ,
  SIZE(WIDTH * HEIGHT * 2)
#endif
{
  errno = 0;
  settings = (CameraSettingsV6) (upper ? CameraSettingsUpperV6() : CameraSettingsV6());
  appliedSettings = settings;
  initOpenVideoDevice(device);

  initSetImageFormat();
  initDefaultControlSettings(flip);
  setFrameRate(1, 30);

  initRequestAndMapBuffers();
  initQueueAllBuffers();

  // Hack to disable auto focus even for Proteus
  setControlSetting(V4L2_CID_FOCUS_AUTO, 0);

  startCapturing();
}

NaoCameraV6::~NaoCameraV6()
{
  // disable streaming
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(ioctl(fd, VIDIOC_STREAMOFF, &type) != -1);

  // unmap buffers
  for (int i = 0; i < frameBufferCount; ++i)
    munmap(mem[i], memLength[i]);

  // close the device
  close(fd);
  free(buf);
}

bool NaoCameraV6::captureNew(NaoCameraV6& cam1, NaoCameraV6& cam2, int timeout, bool& errorCam1, bool& errorCam2)
{
  NaoCameraV6* cams[2] = { &cam1, &cam2 };

  ASSERT(cam1.currentBuf == nullptr);
  ASSERT(cam2.currentBuf == nullptr);

  errorCam1 = errorCam2 = false;

  struct pollfd pollfds[2] =
  {
    {cams[0]->fd, POLLIN | POLLPRI, 0},
    {cams[1]->fd, POLLIN | POLLPRI, 0},
  };
  int polled = poll(pollfds, 2, timeout);
  if (polled < 0)
  {
    OUTPUT_ERROR("Cannot poll for camera images. Reason: " << strerror(errno));
    ASSERT(false);
    return false;
  }
  else if (polled == 0)
  {
    OUTPUT_ERROR("One second passed and there's still no image to read from any camera. Terminating.");
    return false;
  }

  for (int i = 0; i < 2; ++i)
  {
    if (pollfds[i].revents & POLLIN)
    {
      if (ioctl(cams[i]->fd, VIDIOC_DQBUF, cams[i]->buf) == -1)
      {
        OUTPUT_ERROR("VIDIOC_DQBUF failed: " << strerror(errno));
        (i == 0 ? errorCam1 : errorCam2) = true;
      }
      else
      {
        //OUTPUT_ERROR("VIDIOC_DQBUF success revents=" << pollfds[i].revents);
        //ASSERT(buf->bytesused == SIZE);
        cams[i]->currentBuf = cams[i]->buf;
        cams[i]->timeStamp = static_cast<unsigned long long>(cams[i]->currentBuf->timestamp.tv_sec) * 1000000ll + cams[i]->currentBuf->timestamp.tv_usec;

        if (cams[i]->first)
        {
          cams[i]->first = false;
          if (cams[i]->upper)
            printf("upper camera is working\n");
          else
            printf("lower camera is working\n");
        }
      }
    }
    else if (pollfds[i].revents)
    {
      OUTPUT_ERROR("strane poll results: " << pollfds[i].revents);
      (i == 0 ? errorCam1 : errorCam2) = true;
    }
  }
  /*
  if(!success)
  {
    OUTPUT_ERROR("Polling failed.");
    return false;
  }
  */

  return true;
}

bool NaoCameraV6::captureNew(int timeout)
{
  // requeue the buffer of the last captured image which is obsolete now
  ASSERT(currentBuf == nullptr);
  BH_TRACE;

  const unsigned startPollingTimestamp = SystemCall::getCurrentSystemTime();
  struct pollfd pollfd = { fd, POLLIN | POLLPRI, 0 };
  int polled = poll(&pollfd, 1, timeout); // Fail after timeout
  if (polled < 0)
  {
    OUTPUT_ERROR((upper ? "upper " : "lower ") << "camera : Cannot poll. Reason: " << strerror(errno));
    ASSERT(false);
  }
  else if (polled == 0)
  {
    OUTPUT_ERROR((upper ? "upper " : "lower ") << "camera : a lot of time passed and there's still no image to read from the camera. Terminating.");
    return false;
  }
  else if (pollfd.revents & (POLLERR | POLLNVAL))
  {
    OUTPUT_ERROR((upper ? "upper " : "lower ") << "camera : Polling failed.");
    return false;
  }
  // dequeue a frame buffer (this call blocks when there is no new image available) */
  if (ioctl(fd, VIDIOC_DQBUF, buf) == -1)
  {
    OUTPUT_ERROR("VIDIOC_DQBUF failed: " << strerror(errno));
    return false;
  }
  BH_TRACE;
  //ASSERT(buf->bytesused == SIZE);
  currentBuf = buf;
  timeStamp = static_cast<unsigned long long>(currentBuf->timestamp.tv_sec) * 1000000ll + currentBuf->timestamp.tv_usec;
  const unsigned endPollingTimestamp = SystemCall::getCurrentSystemTime();
  timeWaitedForLastImage = endPollingTimestamp - startPollingTimestamp;

  if (first)
  {
    first = false;
    printf("%s camera is working\n", (upper ? "upper" : "lower"));
  }

  return true;
}

void NaoCameraV6::releaseImage()
{
  if (currentBuf)
  {
    VERIFY(ioctl(fd, VIDIOC_QBUF, currentBuf) != -1);
    currentBuf = nullptr;
  }
}

const unsigned char* NaoCameraV6::getImage() const
{
  return currentBuf ? static_cast<unsigned char*>(mem[currentBuf->index]) : nullptr;
}

bool NaoCameraV6::hasImage()
{
  return !!currentBuf; // true <=> currentBuf != 0
  //return false;
}

unsigned long long NaoCameraV6::getTimeStamp() const
{
  if (!currentBuf)
    return 0;
  ASSERT(currentBuf);
  return timeStamp;
}

float NaoCameraV6::getFrameRate() const
{
  return 1.f / 30.f;
}

void NaoCameraV6::setFrameRate(unsigned numerator, unsigned denominator)
{
  // set frame rate
  struct v4l2_streamparm fps;
  memset(&fps, 0, sizeof(struct v4l2_streamparm));
  fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(!ioctl(fd, VIDIOC_G_PARM, &fps));
  fps.parm.capture.timeperframe.numerator = numerator;
  fps.parm.capture.timeperframe.denominator = denominator;
  VERIFY(ioctl(fd, VIDIOC_S_PARM, &fps) != -1);
}

void NaoCameraV6::setSettings(const CameraSettingsV6& settings)
{
  this->settings = settings;
}

void NaoCameraV6::assertCameraSettings()
{
  bool allFine = true;
  // check frame rate
  struct v4l2_streamparm fps;
  memset(&fps, 0, sizeof(fps));
  fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(!ioctl(fd, VIDIOC_G_PARM, &fps));
  if (fps.parm.capture.timeperframe.numerator != 1)
  {
    OUTPUT_ERROR("fps.parm.capture.timeperframe.numerator is wrong.");
    allFine = false;
  }
  if (fps.parm.capture.timeperframe.denominator != 30)
  {
    OUTPUT_ERROR("fps.parm.capture.timeperframe.denominator is wrong.");
    allFine = false;
  }

  // check camera settings
  for (int i = 0; i < CameraSettingsV6::numOfCameraSettings; ++i)
  {
    if (!assertCameraSetting(static_cast<CameraSettingsV6::CameraSetting>(i)))
      allFine = false;
  }

  if (allFine)
  {
    OUTPUT_TEXT("Camera settings match settings stored in hardware/driver.");
  }
}

bool NaoCameraV6::writeCameraSettings()
{

  bool settingsChanged = false;
  // HACK: wait before setting next camera setting
  if (frameCounter > 0)
  {
    --frameCounter;
    return false;
  }
  
  for (int i = 0; i < CameraSettingsV6::numOfCameraSettings; ++i)
  {
    CameraSettingsV6::V4L2Setting& currentSetting = settings.settings[i];
    CameraSettingsV6::V4L2Setting& appliedSetting = appliedSettings.settings[i];
    CameraSettingsV6::CameraSetting setting = static_cast<CameraSettingsV6::CameraSetting>(i);

    for (auto setting : currentSetting.influencingSettings)
    {
      if (setting != CameraSettingsV6::numOfCameraSettings)
      {
        // since this is for autoExposure/autoWhitebalance, 0 means turned off and all other values lead to ignoring influenced settings!
        if (setting == CameraSettingsV6::Gain || setting == CameraSettingsV6::Exposure)
        {
          if (currentSetting.value == 0)
            settings.settings[setting].value = appliedSettings.settings[setting].value;
        }
        else
        {
          if (currentSetting.value > 0)
            settings.settings[setting].value = appliedSettings.settings[setting].value;
        }
      }
    }

    if (currentSetting.value != appliedSetting.value)
    {
      if (!setControlSetting(currentSetting.command, currentSetting.value))
      {
        OUTPUT_WARNING("NaoCameraV6: Setting camera control " << CameraSettingsV6::getName(setting) << " failed for value: " << currentSetting.value << " is: " << appliedSetting.value);
      }
      else
      {
        settingsChanged = true;
        printf("is upper = %d\n %s = %d (%d)\n", upper, CameraSettingsV6::getName(setting), currentSetting.value, appliedSetting.value);
        appliedSetting.value = currentSetting.value;
        frameCounter = 30; // HACK: wait for 30 frames to set next setting
        return false;
      }
    }
  }
  if (settingsChanged)
  {
    assertCameraSettings();


    for (int i = 0; i < CameraSettingsV6::CameraSetting::numOfCameraSettings; i++)
    {
      CameraSettingsV6::V4L2Setting& appliedSetting = appliedSettings.settings[i];
      CameraSettingsV6::V4L2Setting& setting = settings.settings[i];

      setting.value = appliedSetting.value;
    }
  }

  if (appliedSettings.windowPosition != settings.windowPosition
    || appliedSettings.windowSize != settings.windowSize
    || appliedSettings.windowWeights != settings.windowWeights)
  {
    writeAecWindow();
    appliedSettings.windowPosition = settings.windowPosition;
    appliedSettings.windowSize = settings.windowSize;
    appliedSettings.windowWeights = settings.windowWeights;

    frameCounter = 30;
    return false;
  }

  if (appliedSettings.registers != settings.registers)
  {
    updateRegisterIndex = 0;
    updateRegisterIndexByte = 0;
    appliedSettings.registers = settings.registers;
  }

  if (!writeCameraRegisters())
  {
    frameCounter = 5; // wait at least 100ms between registers
    return false;
  }
  
  return true;
}

void NaoCameraV6::readCameraSettings()
{
  for (int i = 0; i < CameraSettingsV6::CameraSetting::numOfCameraSettings; i++)
  {
    CameraSettingsV6::V4L2Setting& appliedSetting = appliedSettings.settings[i];
    CameraSettingsV6::V4L2Setting& setting = settings.settings[i];

    int value = getControlSetting(appliedSetting.command);

    appliedSetting.value = value;
    setting.value = value;
  }

  /*
  for (int i = 0; i < 25; i++) {
  int value = getControlSetting(V4L2_MT9M114_AE_WEIGHT_TABLE_0_0 + i);

  appliedSettings.weights[i] = value;
  settings.weights[i] = value;
  }
  */
}


bool NaoCameraV6::writeCameraRegisters()
{
  if (updateRegisterIndex >= settings.registers.size())
    return true;

  if (updateRegisterIndexByte >= settings.registers[updateRegisterIndex].getSize())
  {
    updateRegisterIndexByte = 0;
    updateRegisterIndex++;
  }

  if (updateRegisterIndex >= settings.registers.size())
    return true;

  const CameraSettingsV6::CameraRegister& reg = settings.registers[updateRegisterIndex];

  uint16_t value = static_cast<uint16_t>((reg.value >> (8 * (reg.getSize() - updateRegisterIndexByte - 1))) & 0xff);
  writeRegister(static_cast<uint16_t>(reg.getAddress() + updateRegisterIndexByte), value);
  updateRegisterIndexByte++;
  
  return false;
}

void NaoCameraV6::doAutoWhiteBalance()
{
  setControlSetting(V4L2_CID_DO_WHITE_BALANCE, 1);
  int value = getControlSetting(V4L2_CID_WHITE_BALANCE_TEMPERATURE);
  if (value > 0)
  {
    OUTPUT_TEXT("New white balance is " << value);
    settings.settings[CameraSettingsV6::WhiteBalance].value = value;
    appliedSettings.settings[CameraSettingsV6::WhiteBalance].value = value;
  }
}

int NaoCameraV6::getControlSetting(unsigned int id)
{
  struct v4l2_queryctrl queryctrl;
  queryctrl.id = id;
  if (ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
  {
    fprintf(stderr, "ioctl to query setting failed for camera setting %d.\n", id);
    return -1;
  }
  if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
  {
    fprintf(stderr, "Camera setting %d is disabled.\n", id);
    return -1; // not available
  }
  if (queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
  {
    fprintf(stderr, "Camera setting %d is unsupported.\n", id);
    return -1; // not supported
  }

  struct v4l2_control control_s;
  control_s.id = id;
  if (ioctl(fd, VIDIOC_G_CTRL, &control_s) < 0)
  {
    fprintf(stderr, "ioctl to retrieve camera setting failed for camera setting %d.\n", id);
    return -1;
  }
  return control_s.value;
}

bool NaoCameraV6::setControlSetting(unsigned int id, int value)
{
  struct v4l2_queryctrl queryctrl;
  queryctrl.id = id;
  if (ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
  {
    OUTPUT_WARNING("NaoCameraV6:  VIDIOC_QUERYCTRL " << id << " call failed (value: " << value << ")!");
    return false;
  }
  if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
  {
    OUTPUT_WARNING("NaoCameraV6:  VIDIOC_QUERYCTRL call failed. Command " << id << " disabled!");
    return false; // not available
  }
  if (queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
  {
    OUTPUT_WARNING("NaoCameraV6:  VIDIOC_QUERYCTRL call failed. Command " << id << " not supported!");
    return false; // not supported
  }
  // clip value
  if (value < queryctrl.minimum)
  {
    OUTPUT_WARNING("NaoCameraV6: Clipping control value. ID: " << id << " to " << queryctrl.minimum);
    value = queryctrl.minimum;
  }
  if (value > queryctrl.maximum)
  {
    value = queryctrl.maximum;
    OUTPUT_WARNING("NaoCameraV6: Clipping control value. ID: " << id << " to " << queryctrl.maximum);
  }
  struct v4l2_control control_s;
  control_s.id = id;
  control_s.value = value;
  if (ioctl(fd, VIDIOC_S_CTRL, &control_s) < 0)
  {
    OUTPUT_WARNING("NaoCameraV6: Setting value ID: " << id << " failed. VIDIOC_S_CTRL return value < 0");
    return false;
  }

  return true;
}

void NaoCameraV6::writeAecWindow()
{
  uvc_xu_control_query xu;
  xu.unit = 3;
  xu.selector = 9;
  xu.query = UVC_SET_CUR;

  std::array<uint8_t,17> data;

  data[0] = 0x10; // enable
  data[1] = settings.windowPosition.x() >> 8; // MSB
  data[2] = settings.windowPosition.x() & 0xff; // LSB
  data[3] = settings.windowPosition.y() >> 8; // MSB
  data[4] = settings.windowPosition.y() & 0xff; // LSB
  data[5] = settings.windowSize.x() >> 8; // MSB
  data[6] = settings.windowSize.x() & 0xff; // LSB
  data[7] = settings.windowSize.y() >> 8; // MSB
  data[8] = settings.windowSize.y() & 0xff; // LSB
  for (int i = 0; i < 8; i++)
  {
    data[i + 9] = static_cast<uint8_t>((settings.windowWeights[i*2] & 0xf) | ((settings.windowWeights[i*2+1] & 0xf) << 4));
  }

  xu.size = data.size();
  xu.data = data.data();

  ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
}

bool NaoCameraV6::setControlSettingFlip(uint16_t value)
{
  uvc_xu_control_query xu;
  xu.unit = 3;
  xu.selector = 12;
  xu.query = UVC_SET_CUR;
  xu.size = sizeof(value);
  xu.data = reinterpret_cast<__u8*>(&value);
  ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
  xu.selector = 13;
  ioctl(fd, UVCIOC_CTRL_QUERY, &xu);

  return true;
}

uint16_t NaoCameraV6::readRegister(uint16_t addr)
{
  uvc_xu_control_query xu_query;
  xu_query.unit = 3;
  xu_query.selector = 0x0e;
  xu_query.query = UVC_SET_CUR;
  xu_query.size = 5;

  uint8_t data[5];
  data[0] = 0; // Read
  data[1] = addr >> 8;
  data[2] = addr & 0xff;
  data[3] = 0;
  data[4] = 0;
  xu_query.data = data;

  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0)
  {
    OUTPUT_ERROR("UVC_SET_CUR fails: " << std::strerror(errno));
  }

  // super ugly, just for testing!
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  xu_query.query = UVC_GET_CUR;
  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0)
  {
    OUTPUT_ERROR("UVC_GET_CUR fails: " << std::strerror(errno));
  }

  return static_cast<uint16_t>( (uint16_t(data[3]) << 8) | uint16_t(data[4]) );
}

void NaoCameraV6::writeRegister(uint16_t addr, uint16_t value)
{
  uvc_xu_control_query xu_query;
  xu_query.unit = 3;
  xu_query.selector = 0x0e;
  xu_query.query = UVC_SET_CUR;
  xu_query.size = 5;

  uint8_t data[5];
  data[0] = 1; // Write
  data[1] = addr >> 8;
  data[2] = addr & 0xff;
  data[3] = value >> 8;
  data[4] = value & 0xff;
  xu_query.data = data;

  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0)
  {
    OUTPUT_ERROR("UVC_SET_CUR fails: " << std::strerror(errno));
  }
}

void NaoCameraV6::outputCurrentValues()
{
  int exposure = (readRegister(0x3500) & 0x0f) << 16 | readRegister(0x3501) << 8 | readRegister(0x3502);
  exposure >>= 4;
  int gain = (readRegister(0x350A) & 0x03) << 8 | readRegister(0x350B);
  int avgReadout = readRegister(0x56A1);

  float exposure_time = exposure / 29460.f;
  OUTPUT_TEXT("Exposure: " << exposure << " (" << exposure_time << "s) Gain: " << gain << " AvgReadout: " << avgReadout);
}

bool NaoCameraV6::assertCameraSetting(CameraSettingsV6::CameraSetting setting)
{
  // check if setting can be ignored
  for (int i = 0; i < CameraSettingsV6::numOfCameraSettings; ++i)
  {
    for (int influence : settings.settings[i].influencingSettings) {
      if (influence == setting) return true;
    }
  }

  int value = getControlSetting(settings.settings[setting].command);
  if (value == settings.settings[setting].value)
    return true;
  else
  {
    OUTPUT_ERROR("Value for command " << settings.settings[setting].command << " (" << CameraSettingsV6::getName(setting) << ") is " << value << " but should be " << settings.settings[setting].value << ".");
    appliedSettings.settings[setting].value = value;
    return false;
  }
}

void NaoCameraV6::initOpenVideoDevice(const char* device)
{
  // open device
  fd = open(device, O_RDWR);
  ASSERT(fd != -1);
}

void NaoCameraV6::initSetImageFormat()
{
  // set format
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(struct v4l2_format));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = WIDTH;
  fmt.fmt.pix.height = HEIGHT;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  VERIFY(!ioctl(fd, VIDIOC_S_FMT, &fmt));

  ASSERT(fmt.fmt.pix.sizeimage == SIZE);
}

void NaoCameraV6::initRequestAndMapBuffers()
{
  // request buffers
  struct v4l2_requestbuffers rb;
  memset(&rb, 0, sizeof(struct v4l2_requestbuffers));
  rb.count = frameBufferCount;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rb.memory = V4L2_MEMORY_MMAP;
  VERIFY(ioctl(fd, VIDIOC_REQBUFS, &rb) != -1);
  ASSERT(rb.count == frameBufferCount);

  // map or prepare the buffers
  buf = static_cast<struct v4l2_buffer*>(calloc(1, sizeof(struct v4l2_buffer)));
  for (int i = 0; i < frameBufferCount; ++i)
  {
    buf->index = i;
    buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf->memory = V4L2_MEMORY_MMAP;
    VERIFY(ioctl(fd, VIDIOC_QUERYBUF, buf) != -1);
    memLength[i] = buf->length;
    mem[i] = mmap(0, buf->length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf->m.offset);
    ASSERT(mem[i] != MAP_FAILED);
  }
}

void NaoCameraV6::initQueueAllBuffers()
{
  // queue the buffers
  for (int i = 0; i < frameBufferCount; ++i)
  {
    buf->index = i;
    buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf->memory = V4L2_MEMORY_MMAP;
    VERIFY(ioctl(fd, VIDIOC_QBUF, buf) != -1);
  }
}

void NaoCameraV6::initDefaultControlSettings(bool flip)
{
  setControlSettingFlip(flip ? 1 : 0);
}

void NaoCameraV6::startCapturing()
{
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(ioctl(fd, VIDIOC_STREAMON, &type) != -1);
}
