// Auto Calibration of Walk and Camera Matrix

option(AutoCalibrate)
{
  initial_state(startCalibration)
  {
    transition
    {
      if (state_time > 3000)
      {
        SystemCall::text2Speech("walk onto field...");
        goto walkOntoField;
      }
    }
    action
    {
      theBehaviorData.behaviorState = BehaviorData::calibrationStarted; // something other than calibrate
      Walk(WalkRequest::speed, 0, 0, 0, WalkRequest::none);
    }
  }

  state(walkOntoField)
  {
    transition
    {
      if (state_time > 6000)
      {
        SystemCall::text2Speech("calibrate body angle...");
        goto calibrateBodyAngle;
      }
    }
    action
    {
      theBehaviorData.behaviorState = BehaviorData::calibrationStarted; // something other than calibrate
      Walk(WalkRequest::speed, 50, 0, 0, WalkRequest::none);
    }
  }

  state(calibrateBodyAngle)
  {
    transition
    {
      // give body angle calibration some time to reset the bodyAngleCalibrated flag after reactivation
      if (state_time > 1000 && theWalkCalibration.bodyAngleCalibrated)
      {
        SystemCall::text2Speech("calibrate camera matrix...");
        goto calibrateCameraMatrix;
      }
    }
    action
    {
      theBehaviorData.behaviorState = BehaviorData::calibrateBody;
      Walk(WalkRequest::speed, 0, 0, 0, WalkRequest::none);
    }
  }

  // TODO: communicate with automatic camera matrix calibration
  state(calibrateCameraMatrix)
  {
    transition
    {
      if (theCMCorrectorStatus.state == CMCorrectorStatus::CalibrationState::finished)
      {
        SystemCall::text2Speech("finished calibration...");
        goto finishedAutoCalibration;
      }
    }
    action
    {
      theBehaviorData.behaviorState = BehaviorData::calibrateCameraMatrix;
      Walk(WalkRequest::speed, 0, 0, 0, WalkRequest::none);
    }
  }

  //state(calibrateWalk)
  //{
  //  transition
  //  {
  //    if (theWalkCalibration.walkCalibrated)
  //      goto finishedAutoCalibration;
  //  }
  //  action
  //  {
  //    theBehaviorData.behaviorState = BehaviorData::calibrateWalk;
  //    Walk(WalkRequest::speed, 0, 0, 0, WalkRequest::none);
  //  }
  //}

  target_state(finishedAutoCalibration)
  {
    transition {}
    action
    {
      theBehaviorData.behaviorState = BehaviorData::game;
    }
  }
}
