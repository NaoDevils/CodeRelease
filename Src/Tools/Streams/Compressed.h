/**
 * @file Compressed.h
 *
 * This contains some compressed basic types and their streaming functions.
 *
 * @author <a href="mailto:aaron.larisch@udo.edu">Aaron Larisch</a>
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include <vector>
#include <limits>

struct Vector2fCompressed : public Vector2f, public Streamable
{
private:
  static constexpr int bits = 12;
  static constexpr float max = (1 << (bits - 1)) - 1;
  static constexpr float min = -(1 << (bits - 1));
  static constexpr float precision = 10.f;

public:
  using Vector2f::Vector2f;
  Vector2fCompressed(const Vector2f& vec) : Vector2f(vec){};
  Vector2fCompressed(Vector2f&& vec) : Vector2f(std::move(vec)){};

  void serialize(In* in, Out* out) override;
};

// We cannot inherit from Angle here. Otherwise, the compiler will convert
// an object of this class to Angle, which doesn't use the custom serialize function.
class AngleCompressed : public Streamable
{
private:
  Angle angle;

public:
  constexpr AngleCompressed() = default;
  constexpr AngleCompressed(float angle) : angle(angle) {}
  constexpr AngleCompressed(Angle angle) : angle(angle) {}

  AngleCompressed& operator=(const Angle& other)
  {
    angle = other;
    return *this;
  }

  operator float&() { return angle; }
  operator Angle&() { return angle; }
  constexpr operator const Angle&() const { return angle; }
  constexpr operator const float&() const { return angle; }

  void serialize(In* in, Out* out) override;
};

struct Pose2fCompressed : public Pose2f
{
  using Pose2f::Pose2f;
  Pose2fCompressed(const Pose2f& pose) : Pose2f(pose){};

  void serialize(In* in, Out* out) override;
};

class ValidityCompressed : public Streamable
{
private:
  float validity;

public:
  constexpr ValidityCompressed() = default;
  constexpr ValidityCompressed(float validity) : validity(validity) {}

  ValidityCompressed& operator=(float other)
  {
    validity = other;
    return *this;
  }

  operator float&() { return validity; }
  constexpr operator const float&() const { return validity; }

  void serialize(In* in, Out* out) override;
};

// Alias for std::vector that allows to implement a different stream function using
// an unsigned char instead of int for the size.
template <class T, class Allocator = std::allocator<T>> class VectorCompressed : public std::vector<T, Allocator>
{
public:
  using std::vector<T, Allocator>::vector;
  VectorCompressed(const std::vector<T>& vec) : std::vector<T>(vec){};
  VectorCompressed(std::vector<T>&& vec) : std::vector<T>(std::move(vec)){};
};

// Alias for std::vector that allows to implement a different stream function using
// an unsigned char instead of int for the size and combining multiple enum values into a single number.
template <typename E, unsigned char Num> class EnumVectorCompressed : public std::vector<E>
{
  static_assert(Num > 0, "Number of elements must be greater than 0");

public:
  using std::vector<E>::vector;
  EnumVectorCompressed(const std::vector<E>& vec) : std::vector<E>(vec){};
  EnumVectorCompressed(std::vector<E>&& vec) : std::vector<E>(std::move(vec)){};
};

// Extension of Streamable.h
namespace Streaming
{
  template <typename E, typename A> struct Streamer<VectorCompressed<E, A>>
  {
    typedef VectorCompressed<E, A> S;
    static void stream(In* in, Out* out, const char* name, S& s, const char* (*enumToString)(int))
    {
      if ((in && !in->isCompressed()) || (out && !out->isCompressed()))
      {
        Streamer<std::vector<E, A>>::stream(in, out, name, static_cast<std::vector<E, A>&>(s), enumToString);
        return;
      }

      registerDefaultElement(s);
      registerWithSpecification(name, typeid(s.data()));
      if (enumToString)
        Streaming::registerEnum(typeid(s[0]), (const char* (*)(int))enumToString);
      if (in)
      {
        in->select(name, -1);
        unsigned char size;
        *in >> size;
        s.resize(size);
        if (!s.empty())
          streamStaticArray(*in, &s[0], s.size() * sizeof(E), enumToString);
        in->deselect();
      }
      else
      {
        out->select(name, -1);
        ASSERT(s.size() <= std::numeric_limits<unsigned char>::max());
        *out << static_cast<unsigned char>(s.size());
        if (!s.empty())
          streamStaticArray(*out, &s[0], s.size() * sizeof(E), enumToString);
        out->deselect();
      }
    }
  };

  template <typename E, unsigned char Num> struct Streamer<EnumVectorCompressed<E, Num>>
  {
    typedef EnumVectorCompressed<E, Num> EVC;
    static void stream(In* in, Out* out, const char* name, EVC& s, const char* (*enumToString)(int))
    {
      if ((in && !in->isCompressed()) || (out && !out->isCompressed()))
      {
        Streamer<std::vector<E>>::stream(in, out, name, static_cast<std::vector<E>&>(s), enumToString);
        return;
      }

      registerDefaultElement(s);
      registerWithSpecification(name, typeid(s.data()));
      if (enumToString)
        Streaming::registerEnum(typeid(s[0]), (const char* (*)(int))enumToString);

      const auto getBytes = [](unsigned char size)
      {
        const double bytes = std::ceil(std::log2(std::pow(Num, size)) / 8.f);
        ASSERT(bytes <= sizeof(unsigned long long));
        return static_cast<unsigned char>(bytes);
      };

      if (in)
      {
        in->select(name, -1);
        unsigned char size;
        *in >> size;
        s.resize(size);

        const unsigned char bytes = getBytes(size);

        unsigned long long val = 0;
        for (char i = 0; i < bytes; ++i)
        {
          unsigned char b;
          *in >> b;

          val |= static_cast<unsigned long long>(b) << i * 8;
        }

        for (auto it = s.rbegin(); it != s.rend(); ++it)
        {
          *it = static_cast<E>(val % Num);
          val /= Num;
        }
        in->deselect();
      }
      else if (out)
      {
        out->select(name, -1);
        ASSERT(s.size() <= std::numeric_limits<unsigned char>::max());
        const unsigned char bytes = getBytes(static_cast<unsigned char>(s.size()));
        unsigned long long val = 0;

        for (const E e : s)
        {
          ASSERT(e < Num);
          val *= Num;
          val += e;
        }

        *out << static_cast<unsigned char>(s.size());
        for (unsigned char i = 0; i < bytes; ++i)
        {
          *out << static_cast<unsigned char>(val & 0xFF);
          val >>= 8;
        }
        out->deselect();
      }
    }
  };
} // namespace Streaming
