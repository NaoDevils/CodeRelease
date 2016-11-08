option(LEDControl)
{
  initial_state(defaultLEDs)
  {
    transition
    {

    }
    action
    {
      // ---- set left eye (ball model) ----
      int timeSinceBallSeenLocal = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
      int timeSinceBallSeenTeamMates = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeenByTeamMate);
      theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::leftEye] = LEDRequest::on;
      if (timeSinceBallSeenLocal < 1500 && timeSinceBallSeenTeamMates < 1500)
        theBehaviorLEDRequest.leftEyeColor = BehaviorLEDRequest::white;
      else if (timeSinceBallSeenLocal < 1500)
        theBehaviorLEDRequest.leftEyeColor = BehaviorLEDRequest::green;
      else if (timeSinceBallSeenTeamMates < 1500)
        theBehaviorLEDRequest.leftEyeColor = BehaviorLEDRequest::red;
      else
      {
        if (theBallModel.timeWhenLastSeen == theFrameInfo.time)
          theBehaviorLEDRequest.leftEyeColor = BehaviorLEDRequest::magenta;
        else
          theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::leftEye] = LEDRequest::off;
      }
      // ---- set right eye (role) ----
      theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::rightEye] = LEDRequest::on;
      switch (theBehaviorData.role)
      {
      case BehaviorData::keeper:
        theBehaviorLEDRequest.rightEyeColor = BehaviorLEDRequest::blue;
        break;
      case BehaviorData::defender:
        theBehaviorLEDRequest.rightEyeColor = BehaviorLEDRequest::green;
        break;
      case BehaviorData::supporterDef:
        theBehaviorLEDRequest.rightEyeColor = BehaviorLEDRequest::white;
        break;
      case BehaviorData::supporterOff:
        theBehaviorLEDRequest.rightEyeColor = BehaviorLEDRequest::magenta;
        break;
      default: //striker
        theBehaviorLEDRequest.rightEyeColor = BehaviorLEDRequest::red;
        break;
      }
      // TODO: ears currently overwritten in LEDHandler!!!
      // ---- set left ear (localization) ----
      if (theSideConfidence.sideConfidence < 0.5f)
        theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::leftEar] = LEDRequest::blinking;
      else
        theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::leftEar] = LEDRequest::on;
      // ---- set right ear (battery+wireless)
      if (theTeammateData.numberOfActiveTeammates == 0)
        theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::rightEar] = LEDRequest::blinking;
      else
        theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::rightEar] = LEDRequest::on;
    }
  }

  
}