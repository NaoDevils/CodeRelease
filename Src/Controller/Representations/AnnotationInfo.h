/**
* @file AnnotationInfo.cpp
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#pragma once

#include "Platform/Thread.h"
#include "Tools/MessageQueue/InMessage.h"

#include <vector>

class AnnotationInfo : public MessageHandler
{
public:
  struct AnnotationData
  {
    unsigned annotationNumber;
    unsigned frame;
    std::string name;
    std::string annotation;
  };

  std::vector<AnnotationData> newAnnotations;
  unsigned timeOfLastMessage;
  unsigned frame = 0;

  AnnotationInfo();
  bool handleMessage(InMessage& message);
  bool handleMessage(InMessage& message, unsigned frameNumber);

  DECLARE_SYNC;
};
