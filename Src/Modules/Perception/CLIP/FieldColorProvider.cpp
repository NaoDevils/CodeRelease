#include "FieldColorProvider.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"

FieldColorProvider::FieldColorProvider()
{
  minYDivFactor = 500;
}

void FieldColorProvider::update(FieldColors& theFieldColor)
{
  MODIFY("module:FieldColorProvider:fieldColorLower", localFieldColorLower);
  MODIFY("module:FieldColorProvider:fieldColorUpper", localFieldColorUpper);

  execute(false);
  theFieldColor = localFieldColorLower;
}


void FieldColorProvider::update(FieldColorsUpper& theFieldColorUpper)
{
  execute(true);
  theFieldColorUpper = localFieldColorUpper;
}

void FieldColorProvider::execute(const bool& upper)
{
  const Image& image = upper ? (Image&)theImageUpper : theImage;
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  if (!image.shouldBeProcessed())
    return;

  if (theFallDownState.state != FallDownState::upright)
    return;

  int minY = 4;

  const Geometry::Line horizon = Geometry::calculateHorizon(cameraMatrix, cameraInfo);

  if (!image.isOutOfImage(horizon.base.x(), horizon.base.y(), 10))
    minY = static_cast<int>(horizon.base.y());

  /* 
  ** Create average field color (over whole image up to horizon).
  ** This field color is the basis for the region based field colors.
  */
  Vector2i lowerLeft(4, image.height - 4);
  Vector2i upperRight(image.width - 4, minY);
  buildSamples(upper, lowerLeft, upperRight, maxPixelCountImageFull);
  FieldColors::FieldColor& fieldColor = upper ? localFieldColorUpper.fieldColorArray[0] : localFieldColorLower.fieldColorArray[0];
  calcFieldColorFromSamples(upper, fieldColor);
  FieldColors& fieldColors = upper ? (FieldColors&)localFieldColorUpper : localFieldColorLower;
  fieldColors.horizonYAvg = minY;
  fieldColors.areaHeight = 4 + (lowerLeft.y() - upperRight.y()) / 3;
  fieldColors.areaWidth = 4 + (upperRight.x() - lowerLeft.x()) / 3;

  // now split image into regions for more specialized field colors
  if (useAreaBasedFieldColor)
  {
    for (int x = 0; x < 3; x++)
    {
      for (int y = 0; y < 3; y++)
      {
        int areaNo = 1 + 3 * y + x;
        FieldColors::FieldColor& fieldColorArea = upper ? localFieldColorUpper.fieldColorArray[areaNo] : localFieldColorLower.fieldColorArray[areaNo];
        lowerLeft.x() = 4 + x * fieldColors.areaWidth;
        upperRight.x() = 4 + x * fieldColors.areaWidth + fieldColors.areaWidth;
        upperRight.y() = minY + y * fieldColors.areaHeight;
        lowerLeft.y() = minY + y * fieldColors.areaHeight + fieldColors.areaHeight;
        buildSamples(upper, lowerLeft, upperRight, maxPixelCountImagePart);
        calcFieldColorFromSamples(upper, fieldColorArea);
      }
    }
    smoothFieldColors(upper);
  }
}

void FieldColorProvider::buildSamples(const bool& upper, const Vector2i& lowerLeft, const Vector2i& upperRight, const int& sampleSize)
{
  const Image& image = upper ? (Image&)theImageUpper : theImage;
  sampleNo = 0;
  int sampleNoMax = sampleSize - 1;
  int scanWidth = upperRight.x() - lowerLeft.x();
  int scanHeight = lowerLeft.y() - upperRight.y();

  for (int i = 0; i < 64; i++)
  {
    histY[i] = 0;
    histCb[i] = 0;
    histCr[i] = 0;
  }

  // build samples
  float imageRatio = (float)scanWidth / (float)(scanHeight);
  int ySamples = (int)sqrt((float)sampleSize / imageRatio);
  int xSamples = (int)((float)ySamples * imageRatio);
  int xStep = (scanWidth / xSamples);
  int yStep = (scanHeight / ySamples);
  for (int x = lowerLeft.x(); x <= upperRight.x(); x += xStep)
  {
    for (int y = upperRight.y(); y <= lowerLeft.y(); y += yStep)
    {
      if (sampleNo < sampleNoMax && !image.isOutOfImage(x, y, 2))
      {
        Image::Pixel p = image[y][x];
        samples[sampleNo].y = p.y;
        samples[sampleNo].cb = p.cb;
        samples[sampleNo].cr = p.cr;
        sampleNo++;
      }
    }
  }
}

void FieldColorProvider::calcFieldColorFromSamples(const bool& upper, FieldColors::FieldColor& fieldColor)
{
  FieldColors::FieldColor& lastMainFieldColor = upper ? localFieldColorUpper.fieldColorArray[0] : localFieldColorLower.fieldColorArray[0];
  maxY = maxCr = maxCb = oldMax = 0;
  int optCr = lastMainFieldColor.fieldColorOptCr;
  int maxFieldColorY = lastMainFieldColor.maxFieldColorY;
  int optY = lastMainFieldColor.fieldColorOptY;


  // find most common cr value and base detection of field color cb/y values on that
  for (int i = 0; i < sampleNo; i++)
  {
    histCr[((int)samples[i].cr) / 4] += fieldColorWeighted(samples[i], optCr, maxFieldColorY);
  }


  // find max cr sum in histograms
  for (int i = 0; i < 64; i++)
  {
    oldMax = maxCr;
    maxCr = std::max(maxCr, histCr[i]);
    if (oldMax < maxCr)
      fieldColor.fieldColorOptCr = i * 4 + 1;
  }

  // ..and find optCr maxDiffs
  int rangeDownCr = 0;
  int rangeUpCr = 0;
  for (int i = (fieldColor.fieldColorOptCr - 1) / 4 + 1; i < 64; i++)
  {
    if (histCr[i] > maxCr / 10)
      rangeUpCr++;
    else
      break;
  }
  for (int i = (fieldColor.fieldColorOptCr - 1) / 4 - 1; i >= 0; i--)
  {
    if (histCr[i] > maxCr / 10)
      rangeDownCr++;
    else
      break;
  }
  fieldColor.fieldColorMaxDistCr = std::min(19, std::max(std::max(rangeUpCr, rangeDownCr) * 4 + 4, 12));
  fieldColor.fieldColorOptCr = (fieldColor.fieldColorOptCr + ((rangeUpCr - rangeDownCr) * 2));

  // find most common cb/y values
  for (int i = 0; i < sampleNo; i++)
  {
    if (std::abs(fieldColor.fieldColorOptCr - (int)samples[i].cr) < fieldColor.fieldColorMaxDistCr)
    {
      histCb[((int)samples[i].cb) / 4] += std::max(150 - (int)samples[i].cb, 0);
      histY[((int)samples[i].y) / 4] += 255 - std::max((int)samples[i].y, std::min(3 * optY / 2, 60));
    }
  }

  // find max cb/y sum in histograms
  for (int i = 0; i < 64; i++)
  {
    oldMax = maxCb;
    maxCb = std::max(histCb[i], maxCb);
    if (oldMax < maxCb)
      fieldColor.fieldColorOptCb = i * 4 + 1;
    oldMax = maxY;
    maxY = std::max(histY[i], maxY);
    if (oldMax < maxY)
      fieldColor.fieldColorOptY = i * 4 + 1;
  }

  // look for the middle value of all good cb values of field color pixels
  int rangeDownCb = 0;
  int rangeUpCb = 0;
  for (int i = (fieldColor.fieldColorOptCb - 1) / 4 + 1; i < 64; i++)
  {
    if (histCb[i] > maxCb / 10)
      rangeUpCb++;
    else
      break;
  }
  for (int i = (fieldColor.fieldColorOptCb - 1) / 4 - 1; i >= 0; i--)
  {
    if (histCb[i] > maxCb / 10)
      rangeDownCb++;
    else
      break;
  }

  fieldColor.fieldColorOptCb = (fieldColor.fieldColorOptCb + ((rangeUpCb - rangeDownCb) * 2));
  fieldColor.fieldColorMaxDistCb = std::max((rangeUpCb + rangeDownCb + 1) * 2, 10);

  // look for the middle value of all good y values of field color pixels
  int rangeDown = 0;
  int rangeUp = 0;
  for (int i = (fieldColor.fieldColorOptY - 1) / 4 + 1; i < 64; i++)
  {
    if (histY[i] > maxY / minYDivFactor)
      rangeUp++;
    else
      break;
  }
  for (int i = (fieldColor.fieldColorOptY - 1) / 4 - 1; i >= 0; i--)
  {
    if (histY[i] > maxY / minYDivFactor)
      rangeDown++;
    else
      break;
  }

  // set field color ranges/ratios for field pixel test depending on optimal values
  fieldColor.fieldColorOptCbCrRatio = (fieldColor.fieldColorOptCb << 10) / (fieldColor.fieldColorOptCr + 1);
  fieldColor.fieldColorMaxCbCrRatioDiff = fieldColor.fieldColorOptCbCrRatio / fieldColor.fieldColorCbCrFactor;
  fieldColor.fieldColorMaxYDiff = std::max((int)((rangeDown + rangeUp + 1) * 2 * fieldColor.fieldColorYDiffFactor), 30);
  fieldColor.fieldColorOptY = (fieldColor.fieldColorOptY + ((rangeUp - rangeDown) * 2));
  fieldColor.maxFieldColorY = fieldColor.fieldColorOptY + fieldColor.fieldColorMaxYDiff;

  if (upper)
    fieldColor.maxFieldColorY += fieldColor.maxFieldColorY / 5;
  else
    fieldColor.maxFieldColorY += fieldColor.maxFieldColorY / 5;

  //tk: temp. removed this as it does not allow for minLineToFieldColorThreshold values < 40
  //fieldColor.lineToFieldColorYThreshold = std::max((std::min(std::min(fieldColor.fieldColorOptY + fieldColor.fieldColorOptY / 3, fieldColor.fieldColorOptY + 40), 255) - fieldColor.fieldColorOptY), minLineToFieldColorThreshold);
  fieldColor.lineToFieldColorYThreshold = minLineToFieldColorThreshold;
}

void FieldColorProvider::smoothFieldColors(const bool& upper)
{
  FieldColors::FieldColor& fieldColorMain = upper ? localFieldColorUpper.fieldColorArray[0] : localFieldColorLower.fieldColorArray[0];
  for (int i = 1; i < 10; i++)
  {
    // TODO: really smooth things by checking surrounding areas
    // TODO: check first if main field color does make sense?
    FieldColors::FieldColor& fieldColorArea = upper ? localFieldColorUpper.fieldColorArray[i] : localFieldColorLower.fieldColorArray[i];
    if (std::abs(fieldColorArea.fieldColorOptCr - fieldColorMain.fieldColorOptCr) > maxDiffOptCr
        || std::abs(fieldColorArea.fieldColorOptCb - fieldColorMain.fieldColorOptCb) > maxDiffOptCb || std::abs(fieldColorArea.fieldColorOptY - fieldColorMain.fieldColorOptY) > maxDiffOptY
        || std::abs(fieldColorArea.fieldColorOptCbCrRatio - fieldColorMain.fieldColorOptCbCrRatio) > maxDiffCbCrRatio)
      fieldColorArea = fieldColorMain;
  }
}

int FieldColorProvider::fieldColorWeighted(const Image::Pixel& p, const int& optCr, const int& fieldColorMaxY)
{
  return (p.y > fieldColorMaxY || p.cr > (std::max(optCr + 10, 150))) ? 0 : std::max(128 - p.cr, 0) + std::abs(optCr - p.cr) / 4;
}

MAKE_MODULE(FieldColorProvider, perception)
