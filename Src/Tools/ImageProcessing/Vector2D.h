/**
* \file Tools/ImageProcessing/Vector2D.h
* Declares a class thats inherits from the vector class and extends this by two dimensional helper functions.
*
* @author <A href=mailto:fabian.rensen@tu-dortmund.de>Fabian Rensen</A>
*/
#pragma once

#include <vector>

template<typename T> class stdVector2D : public std::vector<T> {

private:
  int width;
  int height;

public:

  /**
   * @brief Constructor taking width and height of the 2D vector
   * @param width The width.
   * @param height The height.
   */
  stdVector2D(int width, int height): width(width), height(height)
  {
    this->resize(width*height);
  }

  /**
   * @brief Can be used to address the data as two dimensional
   * @param x X coordinate
   * @param y Y coordinate
   * @return Value at (x,y)
   */
  const T& operator() (int x, int y) const {
    return this->operator [](x + y*width);
  }

  /**
   * @brief Can be used to address the data as two dimensional
   * @param x X coordinate
   * @param y Y coordinate
   * @return Value at (x,y)
   */
  T& operator() (int x, int y) {
    return this->operator [](x + y*width);
  }

  /**
   * @brief Returns the width of the 2D vector object
   * @return A copy of the width.
   */
  int getWidth() const {
    return width;
  }

  /**
   * @brief Returns the height of the 2D vector object
   * @return A copy of the height.
   */
  int getHeight() const {
    return height;
  }

};
