/**
 * @file RefereeGestureProvider.h
 * 
 * @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#include "RefereeGestureProvider.h"

MAKE_MODULE(RefereeGestureProvider, perception);

void RefereeGestureProvider::update(RefereeGesture& refereeGesture)
{

  refereeGesture.gesture = RefereeGesture::NONE;

  Vector2f leftHandPos = theRefereeKeypoints.position.at(6);
  Vector2f rightHandPos = theRefereeKeypoints.position.at(0);
  Vector2f nosePos = theRefereeKeypoints.position.at(3);

  // test skeleton validity
  bool validSkeleton = true;

  if (useSkeletonCheck)
  {
    // basic sanity check using keypoint heights:
    float rightShoulderY = theRefereeKeypoints.position.at(2).y();
    float leftShoulderY = theRefereeKeypoints.position.at(4).y();
    float rightHipY = theRefereeKeypoints.position.at(7).y();
    float leftHipY = theRefereeKeypoints.position.at(8).y();
    float rightKneeY = theRefereeKeypoints.position.at(9).y();
    float leftKneeY = theRefereeKeypoints.position.at(10).y();

    // Nose higher than shoulders
    if (nosePos.y() > rightShoulderY || nosePos.y() > leftShoulderY)
      validSkeleton = false;

    // Hips lower than shoulders
    if (rightHipY < rightShoulderY || rightHipY < leftShoulderY || leftHipY < leftShoulderY || leftHipY < leftShoulderY)
      validSkeleton = false;

    // Knees lower than hips
    if (rightKneeY < rightHipY || rightKneeY < leftHipY || leftKneeY < rightHipY || leftKneeY < leftHipY)
      validSkeleton = false;

    // Ankles lower than knees (currently not used)
  }

  if (useKeypointThreshold)
  {
    // sanity check using nose and hand keypoints confidence
    if (theRefereeKeypoints.confidence.at(0) < keypointThreshold)
      validSkeleton = false;
    if (theRefereeKeypoints.confidence.at(3) < keypointThreshold)
      validSkeleton = false; // nose
    if (theRefereeKeypoints.confidence.at(6) < keypointThreshold)
      validSkeleton = false;

    // Elbows
    if (theRefereeKeypoints.confidence.at(1) < keypointThreshold)
      validSkeleton = false;
    if (theRefereeKeypoints.confidence.at(5) < keypointThreshold)
      validSkeleton = false;
    // Shoulders
    if (theRefereeKeypoints.confidence.at(2) < keypointThreshold)
      validSkeleton = false;
    if (theRefereeKeypoints.confidence.at(4) < keypointThreshold)
      validSkeleton = false;
  }


  if (validSkeleton && leftHandPos.y() < nosePos.y() && rightHandPos.y() < nosePos.y())
  {
    gestureRingBuffer[current_rb] = RefereeGesture::BOTH_HANDS_UP;
  }
  else
  {
    gestureRingBuffer[current_rb] = RefereeGesture::NONE;
  }

  current_rb++;
  if (current_rb >= RINGBUFFERSIZE)
  {
    current_rb = 0;
  }

  int countUp = 0;
  for (int i = 0; i < RINGBUFFERSIZE; i++)
  {
    if (gestureRingBuffer.at(i) == RefereeGesture::BOTH_HANDS_UP)
    {
      countUp++;
    }
  }
  if (countUp >= RINGBUFFERSIZE - 2)
  {
    refereeGesture.gesture = RefereeGesture::BOTH_HANDS_UP;
  }
  else
  {
    refereeGesture.gesture = RefereeGesture::NONE;
  }
}
