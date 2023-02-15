/**
* @file LibTactic.h
*/

class LibTactic : public LibraryBase
{
public:
  /** Constructor for initializing all members*/
  LibTactic();

  void preProcess() override;

  void postProcess() override;

  //Values for Option Playing
  bool blockBall = false;
  bool kickWithOuterFoot = false;
  bool interceptBallPossible = false;

  unsigned timeStampLastWalkKickExecution = 0;
  unsigned timeStampLastDribbleExecution = 0;
};