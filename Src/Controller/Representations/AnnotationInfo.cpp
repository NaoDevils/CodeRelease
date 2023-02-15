/**
* @file AnnotationInfo.cpp
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#include "AnnotationInfo.h"

#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Tools/Global.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/MessageQueue/OutMessage.h"

AnnotationInfo::AnnotationInfo() : timeOfLastMessage(0) {}

bool AnnotationInfo::handleMessage(InMessage& message)
{
  if (message.getMessageID() == idProcessFinished)
  {
    frame++;
    return true;
  }
  else
  {
    return handleMessage(message, frame);
  }
}

bool AnnotationInfo::handleMessage(InMessage& message, unsigned frameNumber)
{
  if (message.getMessageID() == idAnnotation)
  {
    timeOfLastMessage = SystemCall::getCurrentSystemTime();
    {
      SYNC;
      newAnnotations.push_back(AnnotationData());
      AnnotationData& data = newAnnotations.back();

      message.bin >> data.annotationNumber;
      // compatibility: new log files do not contain frame number
      if ((data.annotationNumber & 0x80000000) == 0)
        message.bin >> data.frame;
      data.annotationNumber &= ~0x80000000;
      data.frame = frameNumber;
      message.text >> data.name;
      data.annotation = message.text.readAll();
    }
  }
  return true;
}
