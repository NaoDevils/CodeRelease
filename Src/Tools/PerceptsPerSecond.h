/**
 * \file PerceptsPerSecond.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 *
 * This file declares a ring buffer like data structure which can track the
 * <tt>percepts per second</tt> value over a specific time.
*/

#pragma once

#include <vector>

/**
 * \class PerceptsPerSecond
 *
 * This class declares a ring buffer like data structure which can track the
 * <tt>percepts per second</tt> value over a specific time (\c duration
 * milliseconds).
 *
 * \tparam duration The time (in ms) over which the percepts per second value
 *                  should be tracked.
 */
class PerceptsPerSecond
{
private:
  /// First element: timestamp of the percept (in ms)
  /// Second element: validity of the percept (in range [0,1])
  struct pps_entry : std::pair<unsigned, float> {
    pps_entry(unsigned timestamp, float validity)
    {
      first = timestamp;
      second = validity;
    }
    inline unsigned timestamp() const { return first; }
    inline float validity() const { return second; }
  };
  typedef std::vector<pps_entry> pps_deque;
  /**
   * Saves an timestamp of each percept. The newest percept (largest timestamp)
   * is stored at the front of the double ended queue, the oldest ones at the
   * back. The deque is always orderd. Timestamps older than \c duration
   * milliseconds are removed.
   */
  pps_deque ringBuffer;

  unsigned duration;

public:
  /**
   * \brief Constructor.
   *
   * Creates a new and empty \c PerceptsPerSecond instance.
   */
  PerceptsPerSecond(unsigned duration) : duration(duration) {}

  /**
   * Add a new percept.
   * \param [in] timestamp The time of the new percept (in ms).
   * \param [in] validity Optional: Save validity of the new percept (in range [0,1]).
   *                      Required for computation of mean validity over the last
   *                      \c duration milliseconds (see method \c meanPerceptValidity).
   *                      Default value: 1
   */
  void addPercept(unsigned timestamp, float validity = 1.f)
  {
    addPercept(pps_entry(timestamp, validity));
  }

  /**
   * Copy all percepts from \c source to \c *this.
   * \param [in] source Another \c PerceptsPerSecond object to merge into this one.
   */
  void addPercepts(const PerceptsPerSecond& source)
  {
    //typename pps_deque::iterator it = ringBuffer.begin();
    for (auto it_s = source.ringBuffer.cbegin(); it_s != source.ringBuffer.cend(); ++it_s) {
      // TODO: Verify: This could be very slow.
      addPercept(*it_s);
    }

    //cleanUp(ringBuffer.front());
  }

  /**
   * Return the <tt>percepts per second</tt> value tracked over the last
   * \c duration milliseconds counted from the timestamp of the newest
   * percept.
   * \return Percepts per seconds.
   */
  float pps() const { return static_cast<float>(ringBuffer.size()) * 1000.f / duration; }

  /**
   * Return the <tt>percepts per second</tt> value tracked over the last
   * \c duration milliseconds counted from \c currentTimestamp.
   * \param [in] currentTimestamp The current timestamp (in ms).
   * \return Percepts per seconds.
   */
  float pps(unsigned currentTimestamp)
  {
    cleanUp(currentTimestamp);
    return pps();
  }

  /**
   * Return the mean validity tracked over the last \c duration milliseconds
   * counted from the timestamp of the newest percept.
   * This requires to set the validity parameter of \c addPercept method.
   * \return Mean validity (in range [0,1]).
   */
  float meanPerceptValidity() const
  {
    // Prevent division by 0.
    if (ringBuffer.size() == 0) return 0.f;

    float validitySum = 0.f;
    for (typename pps_deque::const_iterator it = ringBuffer.begin(); it != ringBuffer.end(); ++it) {
      validitySum += it->validity();
    }
    return validitySum / static_cast<float>(ringBuffer.size());
  }

  /**
   * Return the mean validity tracked over the last \c duration milliseconds
   * counted from \c currentTimestamp.
   * This requires to set the validity parameter of \c addPercept method.
   * \param [in] currentTimestamp The current timestamp (in ms).
   * \return Mean validity (in range [0,1]).
   */
  float meanPerceptValidity(unsigned currentTimestamp)
  {
    cleanUp(currentTimestamp);
    return meanPerceptValidity();
  }

  /**
   * Remove all timestamps (percept) which are older than \c duration
   * milliseconds counted from \c currentTimestamp.
   * \param [in] currentTimestamp The current timestamp (in ms).
   */
  void updateCurrentTime(unsigned currentTimestamp) { cleanUp(currentTimestamp); }

private:
  /**
   * Add a new percept entry.
   * \param [in] entry The \c pps_entry to add.
   */
  void addPercept(const pps_entry& entry)
  {
    // Search for the first timestamp smaller than the new one. Start at the end
    typename pps_deque::const_iterator it = ringBuffer.cbegin();
    while (it != ringBuffer.cend() && it->timestamp() > entry.timestamp()) ++it;
    ringBuffer.insert(it, entry);

    auto first = ringBuffer.cbegin();
    if (first != ringBuffer.cend())
      cleanUp(first->timestamp());
  }

  /**
   * Remove all timestamps (percept) from the \c ringBuffer which are older
   * than \c duration milliseconds.
   * \param [in] currentTimestamp The current timestamp (in ms).
   */
  void cleanUp(unsigned currentTimestamp)
  {
    // Do nothing if the current time is less than the duration which should be saved.
    if (currentTimestamp <= duration) return;

    while (ringBuffer.size() > 0 && ringBuffer.back().timestamp() <= currentTimestamp - duration)
      // Remove last element until it is not any more older than duration.
      ringBuffer.pop_back();
  }
};
