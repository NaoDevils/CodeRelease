#include "TeamCommSender.h"

MAKE_MODULE(TeamCommSender, cognitionInfrastructure);

void TeamCommSender::update(TeamCommSenderOutput& teamCommDataSenderOutput)
{
  teamCommDataSenderOutput.dataSent = false;

  if (theTeamCommOutput.sendThisFrame)
  {
    if (size_t size = theTeamCommOutput.data.size(); size <= TeamCommOutput::maximumSize)
      teamCommDataSenderOutput.dataSent = theTeamCommSocket.send(theTeamCommOutput);
    else
      OUTPUT_ERROR("TeamCommSender: Message too big! (" << size << " > " << TeamCommOutput::maximumSize << ")");
  }

  if (teamCommDataSenderOutput.dataSent)
    teamCommDataSenderOutput.dataSentTimestamp = theFrameInfo.time;
}
