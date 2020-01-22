#include "CMCorrector2017.h"
#include "Tools/Math/Transformation.h"
#include "Platform/Common/File.h"

CMCorrector2017::CMCorrector2017()
{
  load();

  state = inactive;
  stateBeginTimeStamp = 0;
}

/**
 * Start camera calibration with given head position (optional).
 */
void CMCorrector2017::start(HeadPosition position)
{
  if(state == inactive || state == finished)
  {
    state = standUp;
  }
  else
  {
    state = capture;
  }

  currentPosition = position;
  stateBeginTimeStamp = theFrameInfo.time;

  // clear old data on restart
  if (position == leftUpper)
  {
    for(int i=0;i<6; i++)
    {
      horizontalLines[i].clear();
    }
    verticalLines[0].clear();
    verticalLines[1].clear();
  }
}

/**
 * Stop camera calibration.
 */
void CMCorrector2017::stop()
{
  if (state == inactive) return;

  if (state == finished)
    state = inactive;
  else
    state = finished;

  stateBeginTimeStamp = theFrameInfo.time;
}

/**
 * Compute camera calibration.
 */
void CMCorrector2017::update(CameraCalibration &cameraCalibration)
{
  // provide camera calibration
  cameraCalibration = localCalibration;
}

/**
 * Set HeadControlRequest to current capture position.
 * Run camera calibration.
 */
void CMCorrector2017::update(HeadControlRequest &headControlRequest)
{
  // run state machine for camera calibration
  runStateMachine();

  switch(state)
  {
  case standUp:
    headControlRequest.controlType = HeadControlRequest::direct;
    headControlRequest.tilt = 0.f;
    headControlRequest.pan = 0.f;
    return;
  case inactive:
    return;
  case finished:
    headControlRequest.controlType = HeadControlRequest::direct;
    headControlRequest.tilt = -38_deg;
    headControlRequest.pan = 0.f;
    return;
  }

  headControlRequest.controlType = HeadControlRequest::direct;

  switch (currentPosition)
  {
  case leftUpper:
  case centerUpper:
  case rightUpper:
    headControlRequest.tilt = 20_deg;
    break;
  case leftLower:
  case centerLower:
  case rightLower:
    headControlRequest.tilt = -20_deg;
    break;
  }

  switch (currentPosition)
  {
  case leftUpper:
  case leftLower:
    headControlRequest.pan = 40_deg;
    break;
  case centerUpper:
  case centerLower:
    headControlRequest.pan = 0.f;
    break;
  case rightUpper:
  case rightLower:
    headControlRequest.pan = -40_deg;
    break;
  }
}

/**
 * Set MotionRequest.
 */
void CMCorrector2017::update(MotionRequest &motionRequest)
{
  if (state == finished)
  {
    motionRequest.motion = MotionRequest::Motion::specialAction;
    motionRequest.specialActionRequest.specialAction = SpecialActionRequest::sitDown;
  }
  else if (state == inactive)
  {
    motionRequest.motion = MotionRequest::Motion::specialAction;
    motionRequest.specialActionRequest.specialAction = SpecialActionRequest::playDead;
  }
  else
  {
    motionRequest.motion = MotionRequest::Motion::specialAction;
    motionRequest.specialActionRequest.specialAction = SpecialActionRequest::stand;
  }
}

/**
 * Set RobotPose to fixed calibration position.
 */
void CMCorrector2017::update(RobotPose &robotPose)
{
  robotPose.rotation = 90_deg;
  robotPose.translation = Vector2f::Zero();
  robotPose.validity = 1.f;
}

/**
 * Set RobotInfo fixed to unpenalized state.
 */
void CMCorrector2017::update(RobotInfo &robotInfo)
{
  robotInfo.number = Global::getSettings().playerNumber;
  robotInfo.naoVersion = Global::getSettings().naoVersion;
  robotInfo.naoBodyType = RobotInfo::NaoType::H25;
  robotInfo.naoHeadType = RobotInfo::NaoType::H25;
  robotInfo.penalty = PENALTY_NONE;
  robotInfo.transitionToBhuman = 0.f;
}

/**
 * Run state machine and do camera calibration.
 */
void CMCorrector2017::runStateMachine()
{
  // register debug responses and drawings
  debug();

  // start / stop camera calibration with chest button
  if (theFrameInfo.getTimeSince(stateBeginTimeStamp) > 1000 && (state == inactive || state == finished || state == standUp) && theKeyStates.pressed[KeyStates::chest]) start();
  else if (theFrameInfo.getTimeSince(stateBeginTimeStamp) > 1000 && theKeyStates.pressed[KeyStates::chest]) stop();

  // state machine
  switch (state)
  {
  case capture:
    // wait 2 sec for stable image
    if (theFrameInfo.getTimeSince(stateBeginTimeStamp) > waitTimeAfterHeadMove && captureData())
    {
      // save sensor data and torso matrix
      headPosition[currentPosition].x() = theJointSensorData.angles[Joints::headYaw];
      headPosition[currentPosition].y() = theJointSensorData.angles[Joints::headPitch];
      torsoMatrix[currentPosition] = theTorsoMatrix;

      // go to next state
      if (currentPosition == rightUpper || currentPosition == centerLower)
      {
        state = calibrate;
      }
      else
      {
        start((HeadPosition)(currentPosition + 1));
      }
    }
    break;
  case calibrate:
    if (currentPosition <= rightUpper && optimizeUpper())
    {
      start(centerLower);
    }
    else if(currentPosition >= leftLower && optimizeLower())
    {
      save();
      stop();
    }
    else
    {
      // repeat if optimization was not successful
      start(leftUpper);
    }
    break;
  }
}

/**
 * Capture field borders and center line at current head position.
 * Return true if data seems valid.
 */
bool CMCorrector2017::captureData()
{
  // go through field line percepts
  for (const CLIPFieldLinesPercept::FieldLine& line : theCLIPFieldLinesPercept.lines)
  {
    // only use lines from upper or lower image
    if (line.fromUpper != (currentPosition <= rightUpper))
      continue;

    Vector2f imageLine = (line.endInImage - line.startInImage).cast<float>();

    // decide between vertical and horizontal lines
    if ((currentPosition == centerUpper || currentPosition == centerLower) && std::abs(imageLine.angle()) < 135_deg && std::abs(imageLine.angle()) > 45_deg)
    {
      // check if line is at least minVLineLength long
      if (verticalLines[currentPosition / 3].size() < minSamples
        && imageLine.norm() > minVLineLength[currentPosition / 3])
      {
        verticalLines[currentPosition / 3].push_back(line);
      }
    }
    else
    {
      // check if line is at least minHLineLength long
      if (horizontalLines[currentPosition].size() < minSamples
        && imageLine.norm() > minHLineLength[currentPosition / 3])
      {
        horizontalLines[currentPosition].push_back(line);
      }
    }
  }

  // return false if there are not enough samples
  if (horizontalLines[currentPosition].size() < minSamples) return false;
  if ((currentPosition == centerUpper || currentPosition == centerLower) && verticalLines[currentPosition / 3].size() < minSamples) return false;

  return true;
}

/**
 * Minimize func within range min to max with given stepSizes.
 */
Vector2a CMCorrector2017::optimizeFunction(const Vector2a& min, const Vector2a& max, const std::vector<Vector2a>& stepSizes, float& bestValue, std::function<float(const Vector2a&)> func)
{
  bestValue = INFINITY;
  Vector2a bestInput(NAN, NAN);

  Vector2a stepMin = min;
  Vector2a stepMax = max;

  float value;
  Vector2a input;
  for (const Vector2a& stepSize : stepSizes)
  {
    for (input.x() = stepMin.x(); input.x() <= stepMax.x(); input.x() += stepSize.x())
    {
      for (input.y() = stepMin.y(); input.y() <= stepMax.y(); input.y() += stepSize.y())
      {
        value = func(input);
        if (value < bestValue)
        {
          bestInput = input;
          bestValue = value;
        }
      }
    }
    stepMin = bestInput - stepSize;
    stepMax = bestInput + stepSize;
  }

  return bestInput;
}

/**
 * Transform given lines from image to robot coordinates depending on camera calibration.
 * Return empty vector, if transformation was not successful.
 */
std::vector<Geometry::Line> CMCorrector2017::linesFromImageToRobot(const std::vector<CLIPFieldLinesPercept::FieldLine>& lines, HeadPosition position, const CameraCalibration& calibration)
{
  RobotCameraMatrix rm;
  rm.computeRobotCameraMatrix(theRobotDimensions, headPosition[position].x(), headPosition[position].y(), calibration, position <= rightUpper);
  CameraMatrix cm;
  cm.computeCameraMatrix(torsoMatrix[position], rm, calibration);

  std::vector<Geometry::Line> result;
  for (const CLIPFieldLinesPercept::FieldLine& line : lines)
  {
    Vector2f startFieldPos, endFieldPos;
    if (!Transformation::imageToRobot(line.startInImage.x(), line.startInImage.y(), cm, position <= rightUpper ? theCameraInfoUpper : theCameraInfo, startFieldPos)) return std::vector<Geometry::Line>();
    if (!Transformation::imageToRobot(line.endInImage.x(), line.endInImage.y(), cm, position <= rightUpper ? theCameraInfoUpper : theCameraInfo, endFieldPos)) return std::vector<Geometry::Line>();

    result.push_back(Geometry::Line(startFieldPos, endFieldPos - startFieldPos));
  }

  return result;
}

/**
 * Print out the error of current camera calibration.
 */
void CMCorrector2017::printCurrentError(HeadPosition position)
{
  OUTPUT_TEXT("Current error " << getName(position) << ": " << calcError(position, localCalibration));
}


/**
 * Optimize body and upper camera rotation correction based on line percepts gathered before.
 * Return true if optimization was successful and error is small enough.
 */
bool CMCorrector2017::optimizeUpper()
{
  printCurrentError(HeadPosition::centerUpper);
  printCurrentError(HeadPosition::leftUpper);
  printCurrentError(HeadPosition::rightUpper);

  CameraCalibration calibration = localCalibration;
  
  /*
   * Optimize body x,y and upper camera z with line percepts from HeadPosition::centerUpper
   */
  calibration.bodyRotationCorrection = Vector2a::Zero();
  calibration.upperCameraRotationCorrection = Vector3a::Zero();

  float bestError = INFINITY;
  Vector2a optim = optimizeFunction(Vector2a(-0.2f, -0.2f), Vector2a(0.2f, 0.2f), { Vector2a(0.05f,0.05f), Vector2a(0.01f,0.01f), Vector2a(0.001f,0.001f) }, bestError, [this,&calibration](const Vector2a& input) {
    // set camera calibration based on input vector
    calibration.bodyRotationCorrection = input;

    return calcError(HeadPosition::centerUpper, calibration);
  });

  if (!verifyError(bestError, HeadPosition::centerUpper)) return false;
  
  // apply result to camera calibration
  calibration.bodyRotationCorrection = optim;


  /*
   * Optimize upper camera x,y with line percepts from HeadPosition::leftUpper and HeadPosition::rightUpper
   */
  const Vector2a sum = calibration.bodyRotationCorrection;
  optim = optimizeFunction(Vector2a(-0.2f, -0.2f), Vector2a(0.2f, 0.2f), { Vector2a(0.05f,0.05f), Vector2a(0.01f,0.01f), Vector2a(0.001f,0.001f) }, bestError, [this,sum,&calibration](const Vector2a& input)
  {
    // set camera calibration based on input vector
    calibration.bodyRotationCorrection = input;
    calibration.upperCameraRotationCorrection.head<2>() = sum - calibration.bodyRotationCorrection;

    return calcError(HeadPosition::leftUpper, calibration) + calcError(HeadPosition::rightUpper, calibration);
  });

  // apply result to camera calibration
  calibration.bodyRotationCorrection = optim.head<2>();
  calibration.upperCameraRotationCorrection.head<2>() = sum - calibration.bodyRotationCorrection;

  if (!verifyError(calcError(HeadPosition::leftUpper, calibration), HeadPosition::leftUpper)) return false;
  if (!verifyError(calcError(HeadPosition::rightUpper, calibration), HeadPosition::rightUpper)) return false;
  
  // apply new camera calibration if everything was successful
  localCalibration = calibration;
  return true;
}

/**
 * Optimize lower camera rotation correction based on line percepts gathered before.
 * Return true if optimization was successful and error is small enough.
 */
bool CMCorrector2017::optimizeLower()
{
  printCurrentError(HeadPosition::centerLower);

  CameraCalibration calibration = localCalibration;

  /*
   * Optimize lower camera x,y with line percepts from HeadPosition::centerLower
   */
  calibration.lowerCameraRotationCorrection = Vector3a::Zero();

  float bestError = INFINITY;
  Vector2a optim = optimizeFunction(Vector2a(-0.2f, -0.2f), Vector2a(0.2f, 0.2f), { Vector2a(0.05f,0.05f), Vector2a(0.01f,0.01f), Vector2a(0.001f,0.001f) }, bestError, [this,&calibration](const Vector2a& input)
  {
    // set camera calibration based on input vector
    calibration.lowerCameraRotationCorrection.head<2>() = input;

    return calcError(HeadPosition::centerLower, calibration);
  });

  if (!verifyError(bestError, HeadPosition::centerLower)) return false;
  
  // apply result to camera calibration
  calibration.lowerCameraRotationCorrection.head<2>() = optim;

  // apply new camera calibration if everything was successful
  localCalibration = calibration;
  return true;
}

/**
 * Calc total error of given HeadPosition and CameraCalibration.
 */
float CMCorrector2017::calcError(HeadPosition position, const CameraCalibration& calibration)
{
  // transform percepts to field
  std::vector<Geometry::Line> horizontalFieldLines = linesFromImageToRobot(horizontalLines[position], position, calibration);
  std::vector<Geometry::Line> verticalFieldLines = linesFromImageToRobot(verticalLines[position / 3], position <= HeadPosition::rightUpper ? HeadPosition::centerUpper : HeadPosition::centerLower, calibration);

  // calculate error
  return getTotalError(horizontalFieldLines, verticalFieldLines);
}

/**
 * Check if error is lower than limit and print out to console.
 */
bool CMCorrector2017::verifyError(float bestError, HeadPosition position)
{
  if (bestError == INFINITY)
  {
    OUTPUT_ERROR(getName(position) << ": Optimization failed");
    return false;
  }

  OUTPUT_TEXT("Best error " << getName(position) << ": " << bestError);

  if (bestError >= errorLimits[position])
  {
    OUTPUT_ERROR("Error " << bestError << " too high!");
    return false;
  }

  return true;
}

/**
 * Calculate total error based on line distances and angles.
 */
float CMCorrector2017::getTotalError(const std::vector<Geometry::Line>& horizontalFieldLines, const std::vector<Geometry::Line>& verticalFieldLines)
{
  if (horizontalFieldLines.size() < 1) return INFINITY;
  if (verticalFieldLines.size() < 1) return INFINITY;

  // push vertical angles into vector
  std::vector<float> verticalAngles;
  for(const Geometry::Line& line : verticalFieldLines) {
    float angle = line.direction.angle();

    // if line is almost vertical, direction is unknown -> substract/add 180_deg if necessary
    if (angle > 90_deg) angle = angle - 180_deg;
    else if (angle < -90_deg) angle = angle + 180_deg;

    verticalAngles.push_back(angle);
  }

  // get median
  std::sort(verticalAngles.begin(), verticalAngles.end());
  float verticalAngle = verticalAngles[verticalAngles.size()/2];

  // push horizontal angles and y distances into vector
  std::vector<float> horizontalAngles, horizontalDistancesStart, horizontalDistancesEnd;
  for (const Geometry::Line& line : horizontalFieldLines)
  {
    // transform field lines depending on robot rotation
    Vector2f lineStart = Transformation::robotToField(Pose2f(90_deg - verticalAngle, 0.f, 0.f), line.base);
    Vector2f lineEnd = Transformation::robotToField(Pose2f(90_deg - verticalAngle, 0.f, 0.f), line.base + line.direction);
    
    horizontalAngles.push_back(line.direction.angle());
    horizontalDistancesStart.push_back(lineStart.y());
    horizontalDistancesEnd.push_back(lineEnd.y());
  }

  // get median
  std::sort(horizontalAngles.begin(), horizontalAngles.end());
  std::sort(horizontalDistancesStart.begin(), horizontalDistancesStart.end());
  std::sort(horizontalDistancesEnd.begin(), horizontalDistancesEnd.end());
  float horizontalAngle = horizontalAngles[horizontalAngles.size() / 2];
  float horizontalDistanceStart = horizontalDistancesStart[horizontalDistancesStart.size() / 2];
  float horizontalDistanceEnd = horizontalDistancesEnd[horizontalDistancesEnd.size() / 2];

  float angleRelativeError = std::abs(std::abs(horizontalAngle) - (90_deg - verticalAngle));

  float distanceErrorFromRobot = std::abs(horizontalDistanceStart - farLineDistance)
    + std::abs(horizontalDistanceEnd - farLineDistance);
  float meanDistanceErrorFromRobot = distanceErrorFromRobot / 2.f;

  return wAngleRelative*angleRelativeError
    + wDistRobot*meanDistanceErrorFromRobot;
}

/**
 * Load camera calibration from file.
 */
void CMCorrector2017::load()
{
  InMapFile stream("./Config/Robots/" + Global::getSettings().robotName + "/" + Global::getSettings().bodyName + "/cameraCalibration.cfg");
  if (stream.exists())
  {
    stream >> localCalibration;
    return;
  }

  // if head-body combination does not exist, combine both configs
  CameraCalibration head, body;
  InMapFile streamHead("./Config/Robots/" + Global::getSettings().robotName + "/" + Global::getSettings().robotName + "/cameraCalibration.cfg");
  if (streamHead.exists())
  {
    streamHead >> head;
  }
  else
    return;
  InMapFile streamBody("./Config/Robots/" + Global::getSettings().bodyName + "/" + Global::getSettings().bodyName + "/cameraCalibration.cfg");
  if (streamBody.exists())
  {
    streamBody >> body;
  }
  else
    return;

  localCalibration.lowerCameraRotationCorrection = head.lowerCameraRotationCorrection;
  localCalibration.upperCameraRotationCorrection = head.upperCameraRotationCorrection;
  localCalibration.bodyRotationCorrection = body.bodyRotationCorrection;
}

/**
 * Save camera calibration to file.
 */
void CMCorrector2017::save()
{
  std::string dir = std::string("mkdir -p ") + std::string(File::getBHDir()) + "/Config/Robots/" + Global::getSettings().robotName + "/" + Global::getSettings().bodyName;
  system(dir.c_str());

  OutMapFile stream("./Config/Robots/" + Global::getSettings().robotName + "/" + Global::getSettings().bodyName + "/cameraCalibration.cfg");
  if (stream.exists())
  {
    stream << localCalibration;
  }
  else
  {
    OUTPUT_ERROR("Saving to config failed!");
  }
}

/**
 * Register debug responses and drawings
 */
void CMCorrector2017::debug()
{
  DECLARE_DEBUG_DRAWING("module:CMCorrector2017:upper", "drawingOnImage");
  COMPLEX_DRAWING("module:CMCorrector2017:upper")
  {
    for (int i = 0; i < 3; i++)
    {
      for (const CLIPFieldLinesPercept::FieldLine& line : horizontalLines[i])
      {
        LINE("module:CMCorrector2017:upper", line.startInImage.x(), line.startInImage.y(), line.endInImage.x(), line.endInImage.y(), 2, Drawings::solidPen, ColorRGBA(255, 0, 0));
      }
    }

    for (const CLIPFieldLinesPercept::FieldLine& line : verticalLines[0])
    {
      LINE("module:CMCorrector2017:upper", line.startInImage.x(), line.startInImage.y(), line.endInImage.x(), line.endInImage.y(), 2, Drawings::solidPen, ColorRGBA(0, 0, 255));
    }
  }

  DECLARE_DEBUG_DRAWING("module:CMCorrector2017:lower", "drawingOnImage");
  COMPLEX_DRAWING("module:CMCorrector2017:lower")
  {
    for (int i = 3; i < 6; i++)
    {
      for (const CLIPFieldLinesPercept::FieldLine& line : horizontalLines[i])
      {
        LINE("module:CMCorrector2017:lower", line.startInImage.x(), line.startInImage.y(), line.endInImage.x(), line.endInImage.y(), 1, Drawings::solidPen, ColorRGBA(255, 0, 0));
      }
    }
    for (const CLIPFieldLinesPercept::FieldLine& line : verticalLines[1])
    {
      LINE("module:CMCorrector2017:lower", line.startInImage.x(), line.startInImage.y(), line.endInImage.x(), line.endInImage.y(), 1, Drawings::solidPen, ColorRGBA(0, 0, 255));
    }
  }

  DEBUG_RESPONSE_ONCE("module:CMCorrector2017:start") start();
  DEBUG_RESPONSE_ONCE("module:CMCorrector2017:stop") stop();
  DEBUG_RESPONSE_ONCE("module:CMCorrector2017:load_config") load();
  DEBUG_RESPONSE_ONCE("module:CMCorrector2017:save_config") save();
}

MAKE_MODULE(CMCorrector2017, perception)