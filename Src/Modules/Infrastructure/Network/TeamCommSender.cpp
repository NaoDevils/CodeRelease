#include "TeamCommSender.h"

MAKE_MODULE(TeamCommSender, cognitionInfrastructure);

void TeamCommSender::update(TeamCommSenderOutput& teamCommDataSenderOutput)
{
  if (theTeamCommOutput.sendThisFrame && theRobotInfo.transitionToFramework == 1.f && theRobotInfo.penalty == PENALTY_NONE)
    teamCommDataSenderOutput.dataSent = theTeamCommSocket.send(theTeamCommOutput);
  else
    teamCommDataSenderOutput.dataSent = false;

  if (teamCommDataSenderOutput.dataSent)
    teamCommDataSenderOutput.dataSentTimestamp = theFrameInfo.time;
}
