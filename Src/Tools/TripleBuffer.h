/**
 * @file TripleBuffer.h
 * Lock-free implementation of triple buffering using std::atomic.
 * Slightly adapted to our needs.
 * @author André Pacheco Neves, Aaron Larisch
 */

//============================================================================
// Name        : TripleBuffer.hxx
// Author      : André Pacheco Neves
// Version     : 1.0 (27/01/13)
// Copyright   : Copyright (c) 2013, André Pacheco Neves
//               All rights reserved.
//
//               Redistribution and use in source and binary forms, with or without
//               modification, are permitted provided that the following conditions are met:
//               	* Redistributions of source code must retain the above copyright
//               	  notice, this list of conditions and the following disclaimer.
//               	* Redistributions in binary form must reproduce the above copyright
//               	  notice, this list of conditions and the following disclaimer in the
//               	  documentation and/or other materials provided with the distribution.
//               	* Neither the name of the <organization> nor the
//               	  names of its contributors may be used to endorse or promote products
//               	  derived from this software without specific prior written permission.
//
//               THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//               ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//               WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//               DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
//               DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//               (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//               LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//               ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//               (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//               SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// Description : Template class for a TripleBuffer as a concurrency mechanism, using atomic operations
// Credits     : http://remis-thoughts.blogspot.pt/2012/01/triple-buffering-as-concurrency_30.html
//============================================================================

#pragma once

#include <atomic>
#include <array>

template <typename T> class TripleBuffer
{

public:
  TripleBuffer<T>();
  TripleBuffer<T>(const T& init);

  // non-copyable behavior
  TripleBuffer<T>(const TripleBuffer<T>&) = delete;
  TripleBuffer<T>& operator=(const TripleBuffer<T>&) = delete;

  const T& readBuffer() const; // get the current snap to read
  T& writeBuffer(); // write a new value
  const T& writeBuffer() const; // const reference to write buffer for dirty read
  bool beginRead(); // swap to the latest value, if any
  void finishWrite(); // flip writer positions dirty / clean

private:
  bool isNewWrite(uint_fast8_t flags); // check if the newWrite bit is 1
  uint_fast8_t swapSnapWithClean(uint_fast8_t flags); // swap Snap and Clean indexes
  uint_fast8_t newWriteSwapCleanWithDirty(uint_fast8_t flags); // set newWrite to 1 and swap Clean and Dirty indexes

  // 8 bit flags are (unused) (new write) (2x dirty) (2x clean) (2x snap)
  // newWrite   = (flags & 0x40)
  // dirtyIndex = (flags & 0x30) >> 4
  // cleanIndex = (flags & 0xC) >> 2
  // snapIndex  = (flags & 0x3)
  mutable std::atomic_uint_fast8_t flags;

  std::array<T, 3> buffer;
};

// include implementation in header since it is a template

template <typename T> TripleBuffer<T>::TripleBuffer()
{
  flags.store(0x6, std::memory_order_relaxed); // initially dirty = 0, clean = 1 and snap = 2
}

template <typename T> TripleBuffer<T>::TripleBuffer(const T& init)
{
  buffer[0] = init;
  buffer[1] = init;
  buffer[2] = init;

  flags.store(0x6, std::memory_order_relaxed); // initially dirty = 0, clean = 1 and snap = 2
}

template <typename T> const T& TripleBuffer<T>::readBuffer() const
{
  return buffer[flags.load(std::memory_order_consume) & 0x3]; // read snap index
}

template <typename T> T& TripleBuffer<T>::writeBuffer()
{
  return buffer[(flags.load(std::memory_order_consume) & 0x30) >> 4]; // write into dirty index
}

template <typename T> const T& TripleBuffer<T>::writeBuffer() const
{
  return buffer[(flags.load(std::memory_order_consume) & 0x30) >> 4]; // write into dirty index
}

template <typename T> bool TripleBuffer<T>::beginRead()
{
  uint_fast8_t flagsNow(flags.load(std::memory_order_consume));
  do
  {
    if (!isNewWrite(flagsNow)) // nothing new, no need to swap
      return false;
  } while (!flags.compare_exchange_weak(flagsNow, swapSnapWithClean(flagsNow), std::memory_order_release, std::memory_order_consume));

  return true;
}

template <typename T> void TripleBuffer<T>::finishWrite()
{
  uint_fast8_t flagsNow(flags.load(std::memory_order_consume));
  while (!flags.compare_exchange_weak(flagsNow, newWriteSwapCleanWithDirty(flagsNow), std::memory_order_release, std::memory_order_consume))
    ;
}

template <typename T> bool TripleBuffer<T>::isNewWrite(uint_fast8_t flags)
{
  // check if the newWrite bit is 1
  return ((flags & 0x40) != 0);
}

template <typename T> uint_fast8_t TripleBuffer<T>::swapSnapWithClean(uint_fast8_t flags)
{
  // swap snap with clean
  return static_cast<uint_fast8_t>((flags & 0x30) | ((flags & 0x3) << 2) | ((flags & 0xC) >> 2));
}

template <typename T> uint_fast8_t TripleBuffer<T>::newWriteSwapCleanWithDirty(uint_fast8_t flags)
{
  // set newWrite bit to 1 and swap clean with dirty
  return static_cast<uint_fast8_t>(0x40 | ((flags & 0xC) << 2) | ((flags & 0x30) >> 2) | (flags & 0x3));
}
