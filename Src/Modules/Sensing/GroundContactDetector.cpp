/**
* @file GroundContactDetector.cpp
* Implementation of module GroundContactDetector.
* @author Colin Graf
* @author Sebastian Hoose
*/

#include "GroundContactDetector.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(GroundContactDetector, sensing)

void GroundContactDetector::update(GroundContactState& groundContactState)
{
  DECLARE_PLOT("module:GroundContactDetector:angleNoiseX");
  DECLARE_PLOT("module:GroundContactDetector:angleNoiseY");
  DECLARE_PLOT("module:GroundContactDetector:accNoiseX");
  DECLARE_PLOT("module:GroundContactDetector:accNoiseY");
  DECLARE_PLOT("module:GroundContactDetector:accNoiseZ");
  DECLARE_PLOT("module:GroundContactDetector:gyroNoiseX");
  DECLARE_PLOT("module:GroundContactDetector:gyroNoiseY");

  //declare plots for footContact
  DECLARE_PLOT("module:GroundContactDetector:frequencyLeft");
  DECLARE_PLOT("module:GroundContactDetector:frequencyRight");
  DECLARE_PLOT("module:GroundContactDetector:leftFootHasGroundContact");
  DECLARE_PLOT("module:GroundContactDetector:rightFootHasGroundContact");
  DECLARE_PLOT("module:GroundContactDetector:currentAvgLeft");
  DECLARE_PLOT("module:GroundContactDetector:currentAvgRight");
  DECLARE_PLOT("module:GroundContactDetector:convLeft");


  MODIFY("module:GroundContactDetector:contact", contact);

  // TODO: BHuman 2015 port
  bool ignoreSensors = (theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction != SpecialActionRequest::standHigh) ||
                       (theMotionRequest.motion == MotionRequest::specialAction && theMotionRequest.specialActionRequest.specialAction != SpecialActionRequest::standHigh);
  if (theMotionInfo.motion == MotionRequest::walk || theMotionInfo.motion == MotionRequest::kick || theMotionInfo.motion == MotionRequest::stand ||
    (theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction != SpecialActionRequest::standHigh) ||
    (theMotionRequest.motion == MotionRequest::specialAction && theMotionRequest.specialActionRequest.specialAction != SpecialActionRequest::standHigh) ||
    (theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction != SpecialActionRequest::stand) ||
    (theMotionRequest.motion == MotionRequest::specialAction && theMotionRequest.specialActionRequest.specialAction != SpecialActionRequest::stand))
  {
	  contact = true;
	  useAngle = false;
	  groundContactState.contact = contact;
	  contactStartTime = theFrameInfo.time;
  }
  if(!ignoreSensors)
  {
    if(contact)
    {
      calibratedAccZValues.push_front(theInertialData.acc.z());

      Vector3f angleDiff = (theTorsoMatrix.rotation * expectedRotationInv).getPackedAngleAxis();
      angleNoises.push_front(Vector2f(sqr(angleDiff.x()), sqr(angleDiff.y())));
      Vector2f angleNoise = angleNoises.average();
      PLOT("module:GroundContactDetector:angleNoiseX", angleNoise.x());
      PLOT("module:GroundContactDetector:angleNoiseY", angleNoise.y());

      if(!useAngle && angleNoises.full() && angleNoise.x() < contactAngleActivationNoise && angleNoise.y() < contactAngleActivationNoise)
        useAngle = true;

      if((useAngle && (angleNoise.x() > contactMaxAngleNoise || angleNoise.y() > contactMaxAngleNoise)) ||
         (calibratedAccZValues.full() && calibratedAccZValues.average() > contactMaxAccZ))
      {
        /*
        if((useAngle && (angleNoise.x > p.contactMaxAngleNoise || angleNoise.y > p.contactMaxAngleNoise)))
          OUTPUT_ERROR("lost ground contact via angle");
        if((calibratedAccZValues.isFull() && calibratedAccZValues.getAverage() > p.contactMaxAccZ))
          OUTPUT_ERROR("lost ground contact via acc");
        */

        contact = false;
        accNoises.clear();
        gyroNoises.clear();
        accValues.clear();
        gyroValues.clear();
        angleNoises.clear();
        if(SystemCall::getMode() == SystemCall::physicalRobot && contactStartTime != 0)
          SystemCall::playSound("high.wav");
      }
    }
    else
    {
      const Vector3f accAverage = accValues.average();
      const Vector2f gyroAverage = gyroValues.average();
      const Vector2f gyro = theInertialData.gyro.head<2>().cast<float>();
      const Vector3f acc = theInertialData.acc.cast<float>();
      accValues.push_front(acc);
      gyroValues.push_front(gyro);
      if(accValues.full())
      {
        accNoises.push_front((acc - accAverage).array().square());
        gyroNoises.push_front((gyro - gyroAverage).array().square());
      }
      Vector3f accNoise = accNoises.average();
      Vector2f gyroNoise = gyroNoises.average();
      PLOT("module:GroundContactDetector:accNoiseX", accNoise.x());
      PLOT("module:GroundContactDetector:accNoiseY", accNoise.y());
      PLOT("module:GroundContactDetector:accNoiseZ", accNoise.z());
      PLOT("module:GroundContactDetector:gyroNoiseX", gyroNoise.x());
      PLOT("module:GroundContactDetector:gyroNoiseY", gyroNoise.y());

      if(accNoises.full() &&
         std::abs(accAverage.z() - Constants::g_1000) < 5.f && std::abs(accAverage.x()) < 5.f && std::abs(accAverage.y()) < 5.f &&
         accNoise.x() < noContactMinAccNoise && accNoise.y() < noContactMinAccNoise && accNoise.z() < noContactMinAccNoise &&
         gyroNoise.x() < noContactMinGyroNoise && gyroNoise.y() < noContactMinGyroNoise)
      {
        contact = true;
        useAngle = false;
        contactStartTime = theFrameInfo.time;
        angleNoises.clear();
        calibratedAccZValues.clear();
      }
    }
  }

  groundContactState.contact = contact;

  expectedRotationInv = theRobotModel.limbs[Limbs::footLeft].translation.z() > theRobotModel.limbs[Limbs::footRight].translation.z() ? theRobotModel.limbs[Limbs::footLeft].rotation : theRobotModel.limbs[Limbs::footRight].rotation;

  //calculate frequencies and foot contact
  float frequencyLeft = 0.f, frequencyRight = 0.f;
  footContact(frequencyLeft,frequencyRight);

  groundContactState.leftFootHasGroundContact = leftHasGroundContact;
  groundContactState.rightFootHasGroundContact = rightHasGroundContact;
  groundContactState.stepFrequencyLeft = frequencyLeft;
  groundContactState.stepFrequencyRight = frequencyRight;
}

void GroundContactDetector::footContact(float& frequencyLeft, float& frequencyRight)
{
  if(!init)
  {
    init = true;
    fsrBufferLeft.fill(0.f);
    fsrBufferRight.fill(0.f);
    frequencyBufferLeft.fill(0.f);
    frequencyBufferRight.fill(0.f);

    convolutionBufferLeft.fill(0.f);
    convolutionBufferRight.fill(0.f);
  }

  //calc convolution with lowpass
  float convLeft = 0.f;
  for(int i = 0; i < (int) convolutionBufferLeft.size();i++)
  {
    convLeft += std::exp(lowpass_a)*convolutionBufferLeft[i];
  }

  float convRight = 0.f;
  for(int i = 0; i < (int) convolutionBufferRight.size();i++)
  {
    convRight += std::exp(lowpass_a)*convolutionBufferRight[i];
  }

  //get max/min values in buffers
  float maxFsrRight = fsrBufferRight.maximum();
  float minFsrRight = fsrBufferRight.minimum();

  float maxFsrLeft = fsrBufferLeft.maximum();
  float minFsrLeft = fsrBufferLeft.minimum();

  //get current average values of each fsr
  float currentLeftAvg = (theFsrSensorData.left[FsrSensorData::fr] + theFsrSensorData.left[FsrSensorData::fl] + theFsrSensorData.left[FsrSensorData::br] + theFsrSensorData.left[FsrSensorData::bl])/4.f;
  float currentRightAvg = (theFsrSensorData.right[FsrSensorData::fr] + theFsrSensorData.right[FsrSensorData::fl] + theFsrSensorData.right[FsrSensorData::br] + theFsrSensorData.right[FsrSensorData::bl])/4.f;

  //calculate errors
  float fsrErrorToCurrentValLeft = (currentLeftAvg - maxFsrLeft < minFsrLeft - currentLeftAvg) ? currentLeftAvg - maxFsrLeft : currentLeftAvg - minFsrLeft;
  float fsrErrorToCurrentValRight = (currentRightAvg - maxFsrRight < minFsrRight - currentRightAvg) ? currentRightAvg - maxFsrRight : currentRightAvg - minFsrRight;

  //calc norm of error
  float fsrErrorToCurrentValLeftNorm = fsrErrorToCurrentValLeft < 0.f ? (-1.f)*fsrErrorToCurrentValLeft : fsrErrorToCurrentValLeft;
  float fsrErrorToCurrentValRightNorm = fsrErrorToCurrentValRight < 0.f ? (-1.f)*fsrErrorToCurrentValRight : fsrErrorToCurrentValRight;

  //decide if foot has contact
  unsigned int timeStampDiff = 0;
  if(fsrErrorToCurrentValLeftNorm > minFsrFootContactThreshold)
  {
    bool oldHasGroundContact = leftHasGroundContact;
    leftHasGroundContact = fsrErrorToCurrentValLeft > 0.f;
    //clear buffer such that a footcontact does not get rerecognized for bufferlength frames
    fsrBufferLeft.clear();
    //calc frequency
    if(leftHasGroundContact != oldHasGroundContact && leftHasGroundContact)
    {
      unsigned int currentTimeStamp = theFrameInfo.time;
      timeStampDiff = currentTimeStamp - lastTimestampLeft;
      frqLeft = 1.f/(static_cast<float>(timeStampDiff)/1000.f);
      lastTimestampLeft = currentTimeStamp;
      frequencyBufferLeft.push_front(frqLeft);
    }
  }

  if(fsrErrorToCurrentValRightNorm > minFsrFootContactThreshold)
  {
    bool oldHasGroundContact = rightHasGroundContact;
    rightHasGroundContact = fsrErrorToCurrentValRight > 0.f;
    //clear buffer such that a footcontact does not get rerecognized for bufferlength frames
    fsrBufferRight.clear();
    //calc frequency
    if(rightHasGroundContact != oldHasGroundContact && rightHasGroundContact)
    {
      unsigned int currentTimeStamp = theFrameInfo.time;
      unsigned int timeStampDiff = currentTimeStamp - lastTimestampRight;
      frqRight = 1.f/(((float)timeStampDiff)/1000.f);
      lastTimestampRight = currentTimeStamp;
      frequencyBufferRight.push_front(frqRight);
    }
  }

  //filter frequencies
  std::vector<float> filterFrequencyListLeft;
  std::vector<float> filterFrequencyListRight;
  for(int i = 0; i < (int) frequencyBufferLeft.size();i++)
  {
    filterFrequencyListLeft.push_back(frequencyBufferLeft[i]);
    filterFrequencyListRight.push_back(frequencyBufferRight[i]);
  }
  std::sort(filterFrequencyListLeft.begin(),filterFrequencyListLeft.end());
  std::sort(filterFrequencyListRight.begin(),filterFrequencyListRight.end());

  //set outputs
  frequencyLeft = filterFrequency && filterFrequencyListLeft.size() > 1 ? filterFrequencyListLeft[(filterFrequencyListLeft.size()/2)] : frqLeft;
  frequencyRight = filterFrequency && filterFrequencyListRight.size() > 1 ? filterFrequencyListRight[filterFrequencyListRight.size()/2] : frqRight;

  //push current sensorvals to buffer
  fsrBufferLeft.push_front(convLeft);
  fsrBufferRight.push_front(convRight);

  convolutionBufferLeft.push_front(currentLeftAvg);
  convolutionBufferRight.push_front(currentRightAvg);

  //plots
  PLOT("module:GroundContactDetector:frequencyLeft",frqLeft);
  PLOT("module:GroundContactDetector:frequencyRight",frqRight);
  PLOT("module:GroundContactDetector:leftFootHasGroundContact",leftHasGroundContact);
  PLOT("module:GroundContactDetector:rightFootHasGroundContact",rightHasGroundContact);
  PLOT("module:GroundContactDetector:currentAvgLeft",currentLeftAvg);
  PLOT("module:GroundContactDetector:currentAvgRight",currentRightAvg);
  PLOT("module:GroundContactDetector:convLeft",convLeft);
  PLOT("module:GroundContactDetector:convRight",convRight);
}
