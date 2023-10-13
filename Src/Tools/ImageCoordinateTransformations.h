class ImageCoordinateTransformations
{
public:
  static void toUpper(const Vector2i& v, Vector2i& vRes)
  {
    vRes.x() = static_cast<int>(v.x() * 2 + .5);
    vRes.y() = static_cast<int>(480 + v.y() * 2 + .5);
  }
};
