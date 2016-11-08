/**
* @file DynamicRingBuffer.h
* @author Marcel Steinbeck
*/

#pragma once
#include <vector>
#include "../Platform/BHAssert.h"

/**
* @class DynamicRingBuffer
*/
template <class V>
class DynamicRingBuffer
{
protected:
  int current;
  int numberOfElements;
  bool cycled;

  int capacity;
  std::vector<V> buffer;
  V sum;

public:
  /** Constructor */
  DynamicRingBuffer(int n)
    : current(n - 1), numberOfElements(0), cycled(false), capacity(n)
  {
    buffer.resize(n);
    ASSERT(n > 0);
    sum = V();
  }

  virtual ~DynamicRingBuffer() {}

  /**
  * adds an entry to the buffer.
  */
  virtual inline void add(const V& v)
  {
    current++;
    cycled = cycled || current >= capacity;
    current %= capacity;
    if (cycled)
      sum -= buffer[current];
    if (++numberOfElements >= capacity) numberOfElements = capacity;
    buffer[current] = v;
    sum += v;
  }

  /**
  * returns an element
  * \parameter i index of entry counting from last added (last=0, last-1=1, ...)
  */
  inline const V& get(int i) const
  {
    return buffer[(capacity + current - i) % capacity];
  }

  /**
  * removes the first added element to the buffer
  */
  virtual inline void removeFirst()
  {
    sum -= buffer[0];
    --numberOfElements;
  }

  /**
  * Returns the number of elements that are currently in the ring buffer
  */
  inline int size() const
  {
    return numberOfElements;
  }

  /**
  * Returns the maximum element count.
  */
  inline int getCapacity() const
  {
    return capacity;
  }

  /**
  * loot at #get
  */
  inline const V& operator[](int i) const
  {
    return buffer[(capacity + current - i) % capacity];
  }

  /**
  * returns the average value of all entries
  * \return the average value
  */
  V getAverage()
  {
    // Return 0 if buffer is empty
    if (0 == this->numberOfElements) return 0;

    return (sum / this->numberOfElements);
  }
};
