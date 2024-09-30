#include "Segmentor.h"
#include "Platform/File.h"
#include "Representations/Infrastructure/CameraInfo.h"

Segmentor::Segmentor()
{
  std::string basePath = std::string(File::getBHDir()) + "/Config/tflite/segmentation/";
  initModel(basePath + modelName, interpreter, model);
}

void Segmentor::execute(tf::Subflow&)
{
  declareDebug();
  if (theFallDownState.state != FallDownState::upright)
  {
    zeroMaps();
    return;
  }
  interpretModel();
  smoothMaps();
  drawMaps();
}

void Segmentor::update(BallHypothesesSegmentor& theBallHypothesesSegmentor)
{
  Stopwatch s("Segmentor-updateBallPercept");
  theBallHypothesesSegmentor.ballSpots.clear();
  Eigen::MatrixXi& map = segmentationMaps[Ball];

  std::vector<std::tuple<Eigen::MatrixXi, float>> components = extractComponents(map, minPercentages[Ball]);

  if (components.size() == 0)
    return;

  for (int i = 0; i < static_cast<int>(components.size()); i++)
  {
    std::tuple<Eigen::MatrixXi, float> componentAndSize = components[i];
    auto& [component, size] = componentAndSize;

    std::tuple<std::vector<Vector2i>, int> convexHullLowerSize = computeConvexHull(component);
    auto& [convexHull, lowerSize] = convexHullLowerSize;
    if (convexHull.size() > 1)
    {
      for (int j = 0; j < static_cast<int>(convexHull.size() - 1); j++)
      {
        Vector2f p1 = toScreenCoordinates(convexHull[j]);
        Vector2f p2 = toScreenCoordinates(convexHull[j + 1]);
        LINE("module:Segmentor:ballPercept", p1.x(), p1.y(), p2.x(), p2.y(), 3, Drawings::solidPen, ColorRGBA::yellow);
      }
      Vector2f p1 = toScreenCoordinates(convexHull[0]);
      Vector2f p2 = toScreenCoordinates(convexHull[convexHull.size() - 1]);
      LINE("module:Segmentor:ballPercept", p1.x(), p1.y(), p2.x(), p2.y(), 3, Drawings::solidPen, ColorRGBA::yellow);

      // compute the mid pt of the convex hull
      Vector2i sum(0, 0);
      for (const Vector2i& x : convexHull)
        sum += x;
      Vector2f mid = sum.cast<float>() / convexHull.size();

      // calculate all distances from the convex hull to the mid pt
      std::vector<float> squaredDists;
      for (const Vector2i& x : convexHull)
      {
        squaredDists.push_back((x.cast<float>() - mid).norm());
      }
      std::vector<float> squaredDists2 = squaredDists;
      std::sort(squaredDists2.begin(), squaredDists2.end());

      // take only the points that have a distance to the mid pt smaller than the median
      float median = squaredDists2[static_cast<int>(squaredDists2.size() * 0.7 + 0.5)];
      std::vector<Vector2i> convexHull2;
      for (size_t j = 0; j < convexHull.size(); j++)
      {
        if (squaredDists[j] <= median)
          convexHull2.push_back(convexHull[j]);
      }

      sum = Vector2i(0, 0);
      for (const Vector2i& x : convexHull2)
        sum += x;
      Vector2i midi = mid.cast<int>();
      mid = toScreenCoordinates(midi);

      CROSS("module:Segmentor:ballPercept", mid.x(), mid.y(), 8, 3, Drawings::solidPen, ColorRGBA::yellow);
      DRAWTEXT("module:Segmentor:ballPercept", mid.x(), mid.y(), 10, ColorRGBA(0, 0, 0, 200), size);
      theBallHypothesesSegmentor.addBallSpot(static_cast<int>(mid.x()), static_cast<int>(mid.y()), false);
      BallSpot& ballSpot = theBallHypothesesSegmentor.ballSpots.back();
      ballSpot.validity = 0.5;
    }
  }
}

void Segmentor::update(RobotsHypothesesSegmentor& theRobotsHypothesesSegmentor)
{
  Stopwatch s("Segmentor-updateRobotPercept");
  theRobotsHypothesesSegmentor.robots.clear();
  Eigen::MatrixXi& map = segmentationMaps[Robot];

  std::vector<std::tuple<Eigen::MatrixXi, float>> components = extractComponents(map, minPercentages[Robot]);

  if (components.size() == 0)
    return;

  for (int i = 0; i < static_cast<int>(components.size()); i++)
  {
    std::tuple<Eigen::MatrixXi, float> componentAndSize = components[i];
    auto& [component, size] = componentAndSize;
    std::tuple<std::vector<Vector2i>, int> convexHullLowerSize = computeConvexHull(component);
    auto& [convexHull, lowerSize] = convexHullLowerSize;
    if (convexHull.size() <= 1)
      return;

    RobotEstimate re;
    // draw convex hull and save to re in relative coordinates
    for (int j = 0; j < static_cast<int>(convexHull.size()) - 1; j++)
    {
      Vector2f p1 = toScreenCoordinates(convexHull[j]);
      Vector2f p2 = toScreenCoordinates(convexHull[j + 1]);
      LINE("module:Segmentor:robotPercept", p1.x(), p1.y(), p2.x(), p2.y(), 3, Drawings::solidPen, ColorRGBA::blue);

      Vector2f p1Robot;
      static_cast<void>(Transformation::imageToRobot(p1, theCameraMatrix, theCameraInfo, p1Robot));
      if (i < lowerSize)
        re.convexHull.push_back(p1Robot);
    }
    Vector2f p1 = toScreenCoordinates(convexHull[0]);
    Vector2f p2 = toScreenCoordinates(convexHull[convexHull.size() - 1]);
    LINE("module:Segmentor:robotPercept", p1.x(), p1.y(), p2.x(), p2.y(), 3, Drawings::solidPen, ColorRGBA::blue);

    Vector2f p1Robot;
    static_cast<void>(Transformation::imageToRobot(p1, theCameraMatrix, theCameraInfo, p1Robot));
    if (i < lowerSize)
      re.convexHull.push_back(p1Robot);

    float leftMostPt = 1000, rightMostPt = -1000, lowestPt = 0, topPt = 1000;
    for (Vector2i& p : convexHull)
    {
      Vector2f pf = toScreenCoordinates(p);
      leftMostPt = std::min(pf.x(), leftMostPt);
      rightMostPt = std::max(pf.x(), rightMostPt);
      topPt = std::min(pf.y(), topPt);
      lowestPt = std::max(pf.y(), lowestPt);
    }
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << size << "";
    DRAWTEXT("module:Segmentor:robotPercept", leftMostPt, lowestPt, 10, ColorRGBA(0, 0, 0, 200), ss.str());

    re.source = RobotEstimate::segmentor;
    re.locationOnField.rotation = 0;
    re.fromUpperImage = false;
    bool largeEnough = size > robotSizeHighValidity;
    re.validity = largeEnough ? 1.0f : 0.5f;
    re.timestampFromImage = theImage.timeStamp;

    // if the percept is near the lower edge of the camera or the body contour, the robot probably doesnt stop there
    int mid = static_cast<int>((leftMostPt + rightMostPt) / 2);
    bool intersectsBodyContour = !theBodyContour.isValidPoint(Vector2i(mid, lowestPt + 5));
    if ((lowestPt + 5 >= theImage.height) || intersectsBodyContour)
    {
      // calculate lower mid of percept in robot coords
      Vector2i lowerMid(mid, lowestPt);
      Vector2f lowerMidToRobot;
      static_cast<void>(Transformation::imageToRobot(lowerMid, theCameraMatrix, theCameraInfo, lowerMidToRobot));
      // shift to be a set amt of mm away
      Vector2f estimate = lowerMidToRobot.normalized() * distanceEstimateMm;

      if (estimate.norm() > lowerMidToRobot.norm())
        estimate = lowerMidToRobot;
      // calculate back to image coordinates
      Vector2f lowestPtEstimate;
      static_cast<void>(Transformation::robotToImage(estimate, theCameraMatrix, theCameraInfo, lowestPtEstimate));
      lowestPt = lowestPtEstimate.y();
      CROSS("module:Segmentor:robotPercept", lowestPtEstimate.x(), lowestPtEstimate.y(), 8, 3, Drawings::solidPen, ColorRGBA::blue);
    }

    re.imageLowerRight = Vector2f(rightMostPt, lowestPt).cast<int>();
    re.imageUpperLeft = Vector2f(leftMostPt, topPt).cast<int>();
    Vector2f boxCenter = (re.imageLowerRight - re.imageUpperLeft).cast<float>() / 2;

    if (Transformation::imageToRobot(Vector2f(boxCenter.x(), re.imageLowerRight.y()), theCameraMatrix, theCameraInfo, re.locationOnField.translation))
    {
      re.distance = re.locationOnField.translation.norm();
      theRobotsHypothesesSegmentor.robots.push_back(re);
    }
  }
}

void Segmentor::zeroMaps()
{
  for (std::map<SegmentationClass, Eigen::MatrixXi>::iterator it = segmentationMaps.begin(); it != segmentationMaps.end(); ++it)
  {
    for (int h = 0; h < mapH; h++)
    {
      for (int w = 0; w < mapW; w++)
      {
        it->second(w, h) = 0;
      }
    }
  }
}

void Segmentor::interpretModel()
{
  Stopwatch s("Segmentor-interpretModel");
  const Image& image = theImage;
  sizeFactorX = theImage.width / (float)mapW;
  sizeFactorY = theImage.height / (float)mapH;

  {
    Stopwatch s1("Segmentor-runModel");
    // get features
    int* inputDims = interpreter->input_tensor(0)->dims->data;
    image.copyAndResizeArea(Vector2i(0, 0), Vector2i(image.width, image.height), {inputDims[2], inputDims[1]}, interpreter->typed_input_tensor<float>(0));

    if (interpreter->Invoke() != kTfLiteOk)
    {
      OUTPUT_ERROR("Failed to invoke tflite!");
      return;
    }
  }

  // outputs
  int output = interpreter->outputs()[0];
  TfLiteIntArray* output_dims = interpreter->tensor(output)->dims;

  // int batchsize = output_dims->data[0];
  int classes = output_dims->data[3];

  float* outputTensor = interpreter->typed_output_tensor<float>(0);

  int ind = 0;
  // outputTensor is a flattened version of (width,height,classes), here we iterate over that
  for (int h = 0; h < mapH; h++)
  {
    for (int w = 0; w < mapW; w++)
    {
      // find the maximum class
      SegmentationClass maxClass = static_cast<SegmentationClass>(0);
      float maxConfidence = 0;
      for (int c = 0; c < classes; c++)
      {
        // reset segmentation map
        SegmentationClass currClass = static_cast<SegmentationClass>(c);
        segmentationMaps[currClass](w, h) = 0;
        float confidence = outputTensor[ind];
        if (confidence > maxConfidence)
        {
          maxConfidence = confidence;
          maxClass = currClass;
        }
        ind++;
      }
      // if background, just take it
      if (maxClass == SegmentationClass::Background)
      {
        segmentationMaps[maxClass](w, h) = 1;
        continue;
      }
      // else, test if the point is in the body contour
      Vector2i coords(w, h);
      Vector2i screenCoords = toScreenCoordinates(coords).cast<int>();
      bool isInBodyContour = !theBodyContour.isValidPoint(screenCoords);
      // accept balls everywhere
      if (maxClass == SegmentationClass::Ball)
        segmentationMaps[maxClass](w, h) = 1;
      // if in body contour, accept only Self
      if (maxClass == SegmentationClass::Self && isInBodyContour)
        segmentationMaps[maxClass](w, h) = 1;
      // if not in body contour, accept rest
      else if (maxClass != SegmentationClass::Self && !isInBodyContour)
        segmentationMaps[maxClass](w, h) = 1;
    }
  }
}

void Segmentor::smoothMaps()
{
  Stopwatch s("Segmentor-smoothMaps");
  for (int c = 1; c <= SegmentationClass::Last; c++)
  {
    SegmentationClass currentClass = static_cast<SegmentationClass>(c);
    Eigen::MatrixXi& map = segmentationMaps[currentClass];
    Eigen::VectorXi colSums = map.rowwise().sum();
    int count = colSums.sum();
    if (count == 0)
      continue;
    bool minPercentageReached = count >= (minPercentages.at(c) * mapW * mapH);
    for (int w = 0; w < mapW; w++)
    {
      if (colSums(w) == 0)
        continue;
      for (int h = 0; h < mapH; h++)
      {
        if (map(w, h) == 0)
          continue;
        if (!minPercentageReached && map(w, h) == 1)
        {
          map(w, h) = 0;
        }
        else
        {
          int n = getNeighbors(map, w, h, minNeighbors);
          if (n < minNeighbors)
          {
            map(w, h) = 0;
            if (useMajorityClass)
            {
              SegmentationClass majorityClass = getMajorityClass(w, h);
              segmentationMaps[majorityClass](w, h) = 1;
            }
          }
        }
      }
    }
  }
}

int Segmentor::getNeighbors(Eigen::MatrixXi& map, int x, int y, int max) const
{
  Stopwatch s("Segmentor-getNeighbors");
  int rows = x < map.cols() - 1 ? 3 : 2;
  rows = x > 0 ? rows : 2;
  int cols = y < map.rows() - 1 ? 3 : 2;
  cols = y > 0 ? cols : 2;
  auto block = map.block(std::max(x - 1, 0), std::max(y - 1, 0), rows, cols);
  return block.sum() - 1;
}

SegmentationClass Segmentor::getMajorityClass(int w, int h)
{
  Stopwatch s("Segmentor-getMajorityClass");
  std::vector<Vector2i> neighborInds;
  SegmentationClass majorityClass = SegmentationClass::Background;
  int majorityCount = -1;
  int totalCount = 0;
  for (int c = 0; c <= SegmentationClass::Last; c++)
  {
    SegmentationClass currClass = static_cast<SegmentationClass>(c);
    Eigen::MatrixXi& map = segmentationMaps[currClass];
    int classCount = getNeighbors(map, w, h, 4);
    totalCount += classCount;
    if (classCount > majorityCount)
    {
      majorityCount = classCount;
      majorityClass = currClass;
    }
    if (8 - totalCount <= majorityCount)
      break;
  }
  return majorityClass;
}

void Segmentor::drawMaps()
{
  Stopwatch s("Segmentor-drawMaps");
  // iterate over all classes except Background and print their map out
  for (int c = 1; c <= SegmentationClass::Last; c++)
  {
    SegmentationClass currClass = static_cast<SegmentationClass>(c);
    ColorRGBA classCol = classColorMap[currClass];
    ColorRGBA col = ColorRGBA(classCol.r, classCol.g, classCol.b, 100);
    Eigen::MatrixXi& map = segmentationMaps[currClass];
    for (int x = 0; x < mapW; x++)
    {
      for (int y = 0; y < mapH; y++)
      {
        if (map(x, y))
        {
          float xCoord = x * sizeFactorX, yCoord = y * sizeFactorY;
          float xCoordP = (x + 1) * sizeFactorX, yCoordP = (y + 1) * sizeFactorY;
          FILLED_RECTANGLE("module:Segmentor:mask:All", xCoord, yCoord, xCoordP, yCoordP, 0, Drawings::solidPen, col, Drawings::solidBrush, col);
          if (currClass == SegmentationClass::Background)
            FILLED_RECTANGLE("module:Segmentor:mask:Background", xCoord, yCoord, xCoordP, yCoordP, 1, Drawings::solidPen, col, Drawings::solidBrush, col);
          if (currClass == SegmentationClass::Field)
            FILLED_RECTANGLE("module:Segmentor:mask:Field", xCoord, yCoord, xCoordP, yCoordP, 1, Drawings::solidPen, col, Drawings::solidBrush, col);
          if (currClass == SegmentationClass::Robot)
            FILLED_RECTANGLE("module:Segmentor:mask:Robot", xCoord, yCoord, xCoordP, yCoordP, 1, Drawings::solidPen, col, Drawings::solidBrush, col);
          if (currClass == SegmentationClass::Line)
            FILLED_RECTANGLE("module:Segmentor:mask:Line", xCoord, yCoord, xCoordP, yCoordP, 1, Drawings::solidPen, col, Drawings::solidBrush, col);
          if (currClass == SegmentationClass::Self)
            FILLED_RECTANGLE("module:Segmentor:mask:Self", xCoord, yCoord, xCoordP, yCoordP, 1, Drawings::solidPen, col, Drawings::solidBrush, col);
          if (currClass == SegmentationClass::Ball)
            FILLED_RECTANGLE("module:Segmentor:mask:Ball", xCoord, yCoord, xCoordP, yCoordP, 1, Drawings::solidPen, col, Drawings::solidBrush, col);
        }
      }
    }
  }
}

void Segmentor::initModel(std::string path, std::unique_ptr<tflite::Interpreter>& interpreter, std::unique_ptr<tflite::FlatBufferModel>& model)
{
  // Load the model
  model = tflite::FlatBufferModel::BuildFromFile(path.c_str());
  TFLITE_MINIMAL_CHECK(model != nullptr);

  // Build the interpreter
  tflite::InterpreterBuilder builder(*model, resolver);
  builder(&interpreter);
  TFLITE_MINIMAL_CHECK(interpreter != nullptr);
  interpreter->SetNumThreads(1);

  // Allocate memory for the tensors
  interpreter->AllocateTensors();

  // Check interpreter state
  // tflite::PrintInterpreterState(featureModelInterpreter.get());
  // initialize segmentation maps
  int* outputDims = interpreter->output_tensor(0)->dims->data;
  mapH = outputDims[1];
  mapW = outputDims[2];
  for (int i = 0; i <= SegmentationClass::Last; i++)
  {
    segmentationMaps[static_cast<SegmentationClass>(i)] = Eigen::MatrixXi::Zero(mapW, mapH);
  }
}

std::vector<Vector2i> Segmentor::grahamScan(Eigen::MatrixXi& map, const bool computeUpperPart)
{
  // This is a graham scan, see https://en.wikipedia.org/wiki/Graham_scan
  std::vector<Vector2i> lowerConvexHull;

  for (int i = 0; i < mapW; i++)
  {
    int x = computeUpperPart ? mapW - 1 - i : i;
    for (int j = mapH - 1; j >= 0; j--)
    {
      int y = computeUpperPart ? mapH - 1 - j : j;
      if (map(x, y))
      {
        Vector2i p3(x, y);
        while (lowerConvexHull.size() >= 1)
        {
          Vector2i p1 = lowerConvexHull[std::max((int)lowerConvexHull.size() - 2, 0)], p2 = lowerConvexHull[lowerConvexHull.size() - 1];
          bool counterClockWise = (p2.x() - p1.x()) * (p3.y() - p1.y()) - (p2.y() - p1.y()) * (p3.x() - p1.x()) <= 0;
          if (!counterClockWise && lowerConvexHull.size() >= 2)
            lowerConvexHull.pop_back();
          else
            break;
        }
        lowerConvexHull.push_back(p3);
        break;
      }
    }
  }
  return lowerConvexHull;
}

std::tuple<std::vector<Vector2i>, int> Segmentor::computeConvexHull(Eigen::MatrixXi& map)
{
  std::vector<Vector2i> lowerConvexHull = grahamScan(map, false);
  std::vector<Vector2i> upperConvexHull = grahamScan(map, true);
  lowerConvexHull.insert(lowerConvexHull.end(), upperConvexHull.begin(), upperConvexHull.end());
  return {lowerConvexHull, static_cast<int>(lowerConvexHull.size())};
}

float Segmentor::dfs(const Eigen::MatrixXi& matrix, Eigen::MatrixXi& component, int x, int y, Eigen::MatrixXi& visited)
{
  std::stack<std::pair<int, int>> stack;
  int size = 1;
  stack.push({x, y});
  int rows = static_cast<int>(matrix.rows());
  int cols = static_cast<int>(matrix.cols());

  while (!stack.empty())
  {
    auto [cx, cy] = stack.top();
    stack.pop();
    component(cx, cy) = 1;

    // Directions for 8 connectivity: N, NE, E, SE, S, SW, W, NW
    const std::vector<std::pair<int, int>> directions = {{-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}};
    for (const auto& [dx, dy] : directions)
    {
      for (const int d : {1, 4})
      {
        int xStep = dx * d;
        int yStep = dy * d;
        int nx = cx + xStep, ny = cy + yStep;
        if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && matrix(nx, ny) == 1 && visited(nx, ny) == 0)
        {
          visited(nx, ny) = 1;
          stack.push({nx, ny});
          size++;
          continue;
        }
      }
    }
  }
  return static_cast<float>(size) / (mapW * mapH);
}

// Extracting each component as a unique MatrixXi
std::vector<std::tuple<Eigen::MatrixXi, float>> Segmentor::extractComponents(const Eigen::MatrixXi& matrix, float minSize)
{
  int rows = static_cast<int>(matrix.rows());
  int cols = static_cast<int>(matrix.cols());
  Eigen::MatrixXi visited = Eigen::MatrixXi::Zero(rows, cols);
  std::vector<std::tuple<Eigen::MatrixXi, float>> components;

  for (int i = 0; i < rows; ++i)
  {
    for (int j = 0; j < cols; ++j)
    {
      if (matrix(i, j) == 1 && visited(i, j) == 0)
      {
        visited(i, j) = 1;
        Eigen::MatrixXi component = Eigen::MatrixXi::Zero(rows, cols);
        float componentSize = dfs(matrix, component, i, j, visited);
        if (componentSize > minSize)
          components.push_back({component, componentSize});
        if (static_cast<int>(components.size()) > maxComponents)
          return components;
      }
    }
  }
  return components;
}


Vector2f Segmentor::toScreenCoordinates(Vector2i& mapCoordinates)
{
  const Vector2f sizeFactor(sizeFactorX, sizeFactorY);
  return (mapCoordinates.cast<float>() + Vector2f(.5, .5)).cwiseProduct(sizeFactor);
}

void Segmentor::declareDebug()
{
  DECLARE_DEBUG_DRAWING("module:Segmentor:ballPercept", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:Segmentor:robotPercept", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:Segmentor:mask:All", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:Segmentor:mask:Background", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:Segmentor:mask:Field", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:Segmentor:mask:Robot", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:Segmentor:mask:Line", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:Segmentor:mask:Self", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:Segmentor:mask:Ball", "drawingOnImage");
}

MAKE_MODULE(Segmentor, perception);
