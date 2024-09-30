// Auto Calibration of Walk and Camera Matrix
Vector2f walkInPosition = Vector2f::Zero();

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
      walkInPosition = Vector2f::Zero();
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
        goto calibrateWalk;
      }
    }
    action
    {
      theBehaviorData.behaviorState = BehaviorData::calibrateCameraMatrix;
      Walk(WalkRequest::speed, 0, 0, 0, WalkRequest::none);
      if (theCMCorrectorStatus.stage == 2 && walkInPosition.isZero() && theRobotPose.validity > 0.8f)
      {
        walkInPosition = theRobotPose.translation;
      }
    }
  }

  state(calibrateWalk)
  {
    transition
    {
      if (action_done || state_time > 5000)
      {
        goto goToWalkInPosition;
      }
    }
    action
    {
      theBehaviorData.behaviorState = BehaviorData::calibrateWalk;
      theHeadControlRequest.controlType = HeadControlRequest::localize;
      Walk(WalkRequest::speed, 10, 10, 2_deg, WalkRequest::none);
    }
  }

  state(goToWalkInPosition)
  {
    transition
    {
      if (action_done || state_time > 25000)
      {
        goto finishedAutoCalibration;
      }
    }
    action
    {
      theBehaviorData.behaviorState = BehaviorData::calibrateWalk;
      GoToFieldCoordinates(Pose2f(theRobotPose.rotation, walkInPosition), 100, 100, 100, 10_deg, true, true);
    }
  }


  target_state(finishedAutoCalibration)
  {
    transition {}
    action
    {
      theBehaviorData.behaviorState = BehaviorData::game;
    }
  }
}
