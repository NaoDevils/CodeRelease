#pragma once

/*
  @file BallSearchAreaProvider.h
  Provides a rectangle shaped area on the field where the robot is supposed to look for the ball

  @author <a href="mailto:maurice.freund@tu-dortmund.de">Maurice Freund</a>
*/

#include "Representations/Modeling/BallSearchArea.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Module/Module.h"

MODULE(BallSearchAreaProvider,
{ ,
  REQUIRES(RoleSymbols),
  REQUIRES(BallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(TeammateData),
  PROVIDES(BallSearchArea),
});

class BallSearchAreaProvider : public BallSearchAreaProviderBase
{
  public:

    void update(BallSearchArea & ballSearchArea);

    

  private:

    bool defenderActive, defSuppActive, offSuppActive, strikerActive;
    
    BallSearchArea localBallSearchArea;
    
    void checkActivityOfTeamMates();

    void determineArea();

    void determineAreaModerat();

    void determineAreaDefensive();

    void determineAreaOffensive();

    void determineAreaModeratDefender();

    void determineAreaModeratDefSupp();

    void determineAreaModeratOffSupp();

    void determineAreaModeratStriker();

    void determineAreaDefensiveDefender();

    void determineAreaDefensiveDefSupp();

    void determineAreaDefensiveOffSupp();

    void determineAreaDefensiveStriker();

    void determineAreaOffensiveDefSupp();

    void determineAreaOffensiveOffSupp();

    void determineAreaOffensiveStriker();
};
