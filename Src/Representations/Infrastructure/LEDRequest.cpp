/**
 * @file Representations/Infrastructure/LEDRequest.cpp
 *
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#include "LEDRequest.h"
#include "Tools/ColorModelConversions.h"

LEDRequest::RGBLED LEDRequest::RGBLED::white{1.f, 1.f, 1.f};
LEDRequest::RGBLED LEDRequest::RGBLED::black{0.f, 0.f, 0.f};
LEDRequest::RGBLED LEDRequest::RGBLED::red{1.f, 0.f, 0.f};
LEDRequest::RGBLED LEDRequest::RGBLED::green{0.f, 1.f, 0.f};
LEDRequest::RGBLED LEDRequest::RGBLED::blue{0.f, 0.f, 1.f};
LEDRequest::RGBLED LEDRequest::RGBLED::yellow{1.f, 1.f, 0.f};
LEDRequest::RGBLED LEDRequest::RGBLED::cyan{0.f, 1.f, 1.f};
LEDRequest::RGBLED LEDRequest::RGBLED::magenta{1.f, 0.f, 1.f};

LEDRequest::RGBLED LEDRequest::RGBLED::orange{1.f, 0.5f, 0.f};
LEDRequest::RGBLED LEDRequest::RGBLED::violet{0.5f, 0.f, 1.f};

LEDRequest::RGBLED LEDRequest::RGBLED::fromHSV(short h, float s, float v)
{
  float r, g, b;
  ColorModelConversions::fromHSVToRGB(h % 360, s, v, r, g, b);
  return LEDRequest::RGBLED(r, g, b);
}
