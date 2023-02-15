#include "TeamCommReceiver.h"

MAKE_MODULE(TeamCommReceiver, cognitionInfrastructure);

void TeamCommReceiver::update(TeamCommInput& teamCommInput)
{
  teamCommInput.messages = theTeamCommSocket.receive();
}
