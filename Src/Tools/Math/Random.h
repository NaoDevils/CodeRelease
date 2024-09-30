/**
 * @file Math/Random.h
 * This class declares some random number functions.
 * @author <a href="mailto:aaorn.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

#include <type_traits>
#include <random>

class Random
{
public:
  template <typename T> static T getNumber(T min, T max);
  template <typename T> static T getNumber(T max);
  static float getNumber();

  static std::default_random_engine& getGenerator() { return generator; }

private:
  static thread_local std::default_random_engine generator;
};

/**
 * The function returns a random integer number in the range of [0..n).
 * @param min the lowest value
 * @param max the upper bound
 * @return The random number.
 */
template <typename T> inline T Random::getNumber(T min, T max)
{
  if constexpr (std::is_integral_v<T>)
  {
    std::uniform_int_distribution<T> rnd(min, max - 1);
    return rnd(generator);
  }
  else
  {
    std::uniform_real_distribution<T> rnd(min, max);
    return rnd(generator);
  }
}

/**
 * The function returns a random integer number in the range of [0..max).
 * @param max the upper bound
 * @return The random number.
 */
template <typename T> inline T Random::getNumber(T max)
{
  return getNumber(T(0), max);
}

/**
 * The function returns a random float in the range of [0..1).
 * @return The random number.
 */
inline float Random::getNumber()
{
  return getNumber(0.f, 1.f);
}


// legacy functions
static inline float randomFloat()
{
  return Random::getNumber();
}
static inline float randomFloat(float min, float max)
{
  return Random::getNumber(min, max);
}
template <typename T> static inline T random(T n)
{
  return Random::getNumber(T(0), n);
}
