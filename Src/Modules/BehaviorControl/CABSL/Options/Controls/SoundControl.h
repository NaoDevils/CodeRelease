option(SoundControl)
{
  common_transition
  {
    if (theGameInfo.state != STATE_INITIAL)
      goto waitForInitial;
  }

  initial_state(waitForInitial)
  {
    transition
    {
      if (theGameInfo.state == STATE_INITIAL)
        goto waitButtonPress1;
    }
    action {}
  }

  state(waitButtonPress1)
  {
    transition
    {
      if (theKeyStates.pressed[KeyStates::Key::headMiddle])
        goto waitButtonRelease1;
    }
    action {}
  }

  state(waitButtonRelease1)
  {
    transition
    {
      if (state_time > 200)
        goto waitButtonPress1;
      if (!theKeyStates.pressed[KeyStates::Key::headMiddle])
        goto waitButtonPress2;
    }
    action {}
  }


  state(waitButtonPress2)
  {
    transition
    {
      if (state_time > 200)
        goto waitButtonPress1;
      if (theKeyStates.pressed[KeyStates::Key::headMiddle])
        goto waitButtonRelease2;
    }
    action {}
  }

  state(waitButtonRelease2)
  {
    transition
    {
      if (state_time > 200)
        goto waitButtonPress1;
      if (!theKeyStates.pressed[KeyStates::Key::headMiddle])
        goto waitButtonPress3;
    }
    action {}
  }


  state(waitButtonPress3)
  {
    transition
    {
      if (state_time > 200)
        goto waitButtonPress1;
      if (theKeyStates.pressed[KeyStates::Key::headMiddle])
        goto waitButtonRelease3;
    }
    action {}
  }

  state(waitButtonRelease3)
  {
    transition
    {
      if (state_time > 200)
        goto waitButtonPress1;
      if (!theKeyStates.pressed[KeyStates::Key::headMiddle])
        goto say;
    }
    action {}
  }


  state(say)
  {
    transition
    {
      goto waitButtonPress1;
    }
    action
    {
      std::string text = "Number: " + std::to_string(theRobotInfo.number) + ". " + "Team: " + std::to_string(theOwnTeamInfo.teamNumber) + ". "
          + "Port: " + std::to_string(theOwnTeamInfo.teamPort) + ". ";

      if (theUSBSettings.updateTimestamp > 0)
      {
        if (!theUSBSettings.wifiSSID.empty())
          text += "Wifi: " + theUSBSettings.wifiSSID + ". ";

        if (!theUSBSettings.ip.empty())
        {
          std::string ip = theUSBSettings.ip;

          // remove subnet suffix
          const size_t pos = ip.find('/');
          if (pos != std::string::npos)
            ip = ip.substr(0, pos);

          // replace . with "dot"
          for (size_t i = 0; (i = ip.find('.', i)) != std::string::npos;)
            ip.replace(i, 1, " dot ");

          text += "I P: " + ip + ". ";
        }

        switch (theNetworkStatus.status)
        {
        case NetworkStatus::Status::configuring:
          text += "configuration in progress. ";
          break;
        case NetworkStatus::Status::configured:
          text += "configuration successful. ";
          break;
        case NetworkStatus::Status::failed:
          text += "configuration failed. ";
          break;
        }

        if ((theUSBStatus.status == USBStatus::MountStatus::readOnly || theUSBStatus.status == USBStatus::MountStatus::readWrite)
            && theUSBStatus.mountTimestamp <= theFieldDimensions.lastUpdate)
          text += "Field dimensions loaded.";
      }
      SystemCall::text2Speech(text);
    }
  }
}
