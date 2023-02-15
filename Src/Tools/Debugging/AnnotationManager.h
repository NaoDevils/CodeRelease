/**
 * @file AnnotationManager.h
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */

#pragma once

#include "Tools/MessageQueue/MessageQueue.h"
#include <mutex>


class AnnotationManager
{
private:
  MessageQueue outData;
  unsigned annotationCounter = 0;
  unsigned lastGameState;
  unsigned lastSetPlay;
  unsigned lastPenalty;
  unsigned lastOwnScore;
  unsigned lastOpponentScore;
  float lastTransitionToFramework;

  // protect MessageQueue
  std::mutex mutex;

  friend class Process;

  AnnotationManager(); // private so only Process can access it.

public:
  void signalProcessStart();
  void clear();

  void addAnnotation();
  void endAnnotation();
  MessageQueue& getOut();
};
