/**
 * The file declares a ring buffer that can determine the sum and average of its
 * elements in constant time. The type of the elements must be assignable, copyable,
 * and must support addition with itself and division by an integral (average()) or
 * floating point (averagef()) number. The interface of the class is similar to
 * types of the standard template library and it also supports for-each loops.
 * @author Thomas Röfer
 */

#pragma once

#include "RingBuffer.h"
#include <type_traits>

class Angle;

template <typename T, std::size_t n = 0> class RingBufferWithSum : public RingBuffer<T, n>
{
private:
  T zero; /**< Sum when the buffer is empty. */
  T currentSum; /**< Sum of current round since index 0. */
  T prevSum; /** Sum of previous round. */

public:
  /**
   * Constructor for a buffer of entries with a default constructor that creates a
   * zero element.
   * @param capacity The maximum number of entries the buffer can store. If not specified,
   *                 the second template parameter is used as default capacity.
   */
  RingBufferWithSum(std::size_t capacity = n) :
    RingBuffer<T, n>(capacity),
    zero()
  {
    static_assert(std::is_same<Angle, T>::value ||
                  std::is_floating_point<T>::value ||
                  std::is_integral<T>::value,
                  "The standard constructor is not applicable for non-standard types in the template argument");
    clear();
  }

  /**
   * Constructor for a buffer of entries without a default constructor that creates a
   * zero element, e.g. Eigen vectors.
   * @param zero A value that is used as zero element.
   * @param capacity The maximum number of entries the buffer can store. If not specified,
   *                 the second template parameter is used as default capacity.
   */
  RingBufferWithSum(const T& zero, std::size_t capacity = n) :
    RingBuffer<T, n>(capacity),
    zero(zero)
  {
    static_assert(!std::is_same<Angle, T>::value &&
                  !std::is_floating_point<T>::value &&
                  !std::is_integral<T>::value, "Use the standard constructor instead!");
    clear();
  }

  /**
   * Copy constructor.
   * If the buffer is full, the previous sum contains all data, otherwise the current
   * sum contains all data.
   * @param other The buffer that is copied.
   */
  RingBufferWithSum(const RingBufferWithSum& other) :
    zero(other.zero),
    currentSum(RingBuffer<T, n>::full() ? other.zero : other.prevSum + other.currentSum),
    prevSum(RingBuffer<T, n>::full() ? other.prevSum + other.currentSum : other.zero)
  {}

  /**
   * Assignment operator.
   * If the buffer is full, the previous sum contains all data, otherwise the current
   * sum contains all data.
   * @param other The buffer that is copied.
   * @return This buffer.
   */
  RingBufferWithSum& operator=(const RingBufferWithSum& other)
  {
    RingBuffer<T, n>::operator=(other);
    zero = other.zero;
    currentSum = RingBuffer<T, n>::full() ? other.zero : other.prevSum + other.currentSum;
    prevSum = RingBuffer<T, n>::full() ? other.prevSum + other.currentSum : other.zero;
    return *this;
  }

  /** Empties the buffer. */
  void clear()
  {
    RingBuffer<T, n>::clear();
    currentSum = prevSum = zero;
  }

  /**
   * Changes the capacity of the buffer. If it actually changes, the complexity is O(size()).
   * @param capacity The maximum number of entries the buffer can store.
   */
  void reserve(std::size_t capacity)
  {
    while(RingBuffer<T, n>::size() > capacity)
      pop_back();
    RingBuffer<T, n>::reserve(capacity);
  }

  /**
   * Adds a new entry to the front of the buffer. The new entry is accessible under
   * index 0, front(), and *begin(). If the buffer was already full, the entry at
   * back() is lost.
   * @param value The value that is added to the buffer.
   */
  void push_front(const T& value)
  {
    if(RingBuffer<T, n>::full())
      prevSum -= RingBuffer<T, n>::back();

    currentSum += value;
    RingBuffer<T, n>::push_front(value);

    // Prevent propagating errors from one round to another
    if(RingBuffer<T, n>::cycled())
    {
      prevSum = currentSum;
      currentSum = zero;
    }
  }
  
  /**
   * Replaces the entry at the front of the buffer. The new entry is still accessible under
   * index 0, front(), and *begin(). All other entries are unchanged.
   * @param newValue The value that is set to the front of the buffer.
   */
  void set_front(const T& newValue)
  {
    if(RingBuffer<T, n>::cycled())
      prevSum += - RingBuffer<T, n>::front() + newValue;
    else
      currentSum += - RingBuffer<T, n>::front() + newValue;
    
    RingBuffer<T, n>::front() = newValue;
  }

  /** Removes the entry back() from the buffer. */
  void pop_back()
  {
    prevSum -= RingBuffer<T, n>::back();
    RingBuffer<T, n>::pop_back();
  }

  /** Returns the sum of all entries in O(1). */
  T sum() const {return prevSum + currentSum;}

  /**
   * Returns the minimum of all entries in O(size()).
   * If the buffer is empty, the zero element is returned.
   */
  T minimum() const
  {
    if(RingBuffer<T, n>::empty())
      return zero;
    else
    {
      T min = RingBuffer<T, n>::front();
      for(const T& t : *this)
        if(t < min)
          min = t;
      return min;
    }
  }

  /**
   * Returns the maximum of all entries in O(size()).
   * If the buffer is empty, the zero element is returned.
   */
  T maximum() const
  {
    if(RingBuffer<T, n>::empty())
      return zero;
    else
    {
      T max = RingBuffer<T, n>::front();
      for(const T& t : *this)
        if(t > max)
          max = t;
      return max;
    }
  }

  /**
   * Returns the average of all entries in O(1). If the buffer is empty, the
   * zero element is returned.
   */
  T average() const;

  /**
   * Returns the average of all entries in O(1) as a float. If the buffer is empty, the
   * zero element is returned. This method is only useful if T is an integral type.
   */
  float averagef() const;

  /**
  * fills/overwrites the buffer with given value
  * @param value value to fill the buffer with
  */
  void fill(T value)
  {
    for (std::size_t i = 0; i < n; i++)
      push_front(value);
  }

  /**
  * returns the variance of all entries
  * \return the variance
  */
  inline T getVariance()
  {
    // Return 0 if buffer is empty or has just one element
    if (RingBuffer<T, n>::size() < 2) return T();
    T avg = average();
    T var = 0;

    for (const T& t : *this)
    {
      var += (t - avg) * (t - avg);
    }

    return (1.f / (RingBuffer<T, n>::size() - 1)) * var;
  }
};

#ifdef WINDOWS // The division might force a type conversion that looses precision
#pragma warning(push)
#pragma warning(disable: 4244 4267)
#elif defined __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif

template<typename T, std::size_t n> T RingBufferWithSum<T, n>::average() const
{
  if(RingBuffer<T, n>::empty())
    return zero;
  else
    return static_cast<T>(sum() / RingBuffer<T, n>::size());
}

template<typename T, std::size_t n> float RingBufferWithSum<T, n>::averagef() const
{
  static_assert(std::is_integral<T>::value, "Use average() instead");
  if(RingBuffer<T, n>::empty())
    return zero;
  else
    return sum() / static_cast<float>(RingBuffer<T, n>::size());
}

#ifdef WINDOWS
#pragma warning(pop)
#elif defined __clang__
#pragma clang diagnostic pop
#endif
