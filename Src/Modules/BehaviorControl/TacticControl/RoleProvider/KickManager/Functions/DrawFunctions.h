#include "Modules/BehaviorControl/TacticControl/RoleProvider/Logs/KickDrawings.h"
#include "Representations/Modeling/HeatMap.h"

class DrawFunctions
{

public:
  static void drawInGrid(const std::vector<Vector2f>& draw_targets, const std::vector<float>& draw_scores, const FieldDimensions& theFieldDimensions)
  {
    COMPLEX_DRAWING(DRAW_EXECUTABLE_KICKS_IN_GRID)
    {
      const bool STRETCH = false;

      std::vector<int> draw_indexes = {};
      for (const Vector2f& draw_target : draw_targets)
      {
        const int draw_index = HeatMap::fieldToIndex(draw_target, theFieldDimensions);
        draw_indexes.push_back(draw_index);
      }

      std::vector<int> unique_draw_indexes = {};
      std::vector<float> unique_draw_scores = {};
      for (size_t i = 0; i < draw_indexes.size(); i++)
      {
        const int index = draw_indexes.at(i);
        const float score = draw_scores.at(i);

        bool foundEqualTarget = false;
        for (size_t j = 0; j < unique_draw_indexes.size(); j++)
        {
          const int uniqueIndex = unique_draw_indexes.at(j);
          const float uniqueScore = unique_draw_scores.at(j);

          if (index == uniqueIndex)
          {
            if (score > uniqueScore)
            {
              unique_draw_scores.at(j) = score;
            }
            foundEqualTarget = true;
            break;
          }
        }
        if (!foundEqualTarget)
        {
          unique_draw_indexes.push_back(index);
          unique_draw_scores.push_back(score);
        }
      }

      if (STRETCH)
      {
        MathUtils::stretch(unique_draw_scores);
      }

      const size_t size = unique_draw_scores.size();
      for (size_t i = 0; i < size; ++i)
      {
        const Vector2f draw_target = HeatMap::indexToField(unique_draw_indexes[i], theFieldDimensions);
        float draw_score = unique_draw_scores[i];
        if (!STRETCH)
        {
          draw_score = (MathUtils::clamp_f(draw_score, -1.5f, 1.5f) + 1.5f) / 3.f;
        }
        CIRCLE(DRAW_EXECUTABLE_KICKS_IN_GRID, draw_target.x(), draw_target.y(), 50, 0, Drawings::noPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA((char)(255 * (1 - draw_score)), (char)(255 * draw_score), 0));
      }
    }
  }

  static void drawFree(const std::vector<Vector2f>& draw_targets, std::vector<float>& draw_scores)
  {
    COMPLEX_DRAWING(DRAW_EXECUTABLE_KICKS_FREELY)
    {
      const bool STRETCH = true;

      if (STRETCH)
      {
        MathUtils::stretch(draw_scores);
      }

      const size_t size = draw_targets.size();
      for (size_t i = 0; i < size; ++i)
      {
        const Vector2f& draw_target = draw_targets.at(i);
        float draw_score = draw_scores.at(i);
        if (!STRETCH)
        {
          draw_score = (MathUtils::clamp_f(draw_score, -1.5f, 1.5f) + 1.5f) / 3.f;
        }
        CIRCLE(DRAW_EXECUTABLE_KICKS_FREELY, draw_target.x(), draw_target.y(), 30, 0, Drawings::noPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA((char)(255 * (1 - draw_score)), (char)(255 * draw_score), 0));
      }
    }
  }

  static void drawKickMinObstacleWidth(const Vector2f& ballPosition, const Vector2f& targetPosition, const float width)
  {
    Vector2f w = Vector2f(targetPosition - ballPosition).normalized() * width;
    w = w.rotate(90_deg);

    const Vector2f p11 = ballPosition - w;
    const Vector2f p12 = ballPosition + w;

    const Vector2f p21 = targetPosition - w;
    const Vector2f p22 = targetPosition + w;

    LINE(DRAW_KICK_MIN_WIDTH, p11.x(), p11.y(), p21.x(), p21.y(), 10, Drawings::solidPen, ColorRGBA::orange);
    LINE(DRAW_KICK_MIN_WIDTH, p12.x(), p12.y(), p22.x(), p22.y(), 10, Drawings::solidPen, ColorRGBA::orange);
  }
};