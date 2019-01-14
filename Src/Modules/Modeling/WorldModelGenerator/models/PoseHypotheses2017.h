/**
* @file PoseHypotheses2017.h
*
* This class represents a collection of PoseHypothesis.
* It secures that there is no memory leakage.
*
* @author <a href="mailto:stefan.tasse@tu-dortmund.de">Stefan Tasse</a>
* @author <a href="mailto:dino.menges@tu-dortmund.de">Dino Menges</a>
*/

#ifndef __PoseHypotheses2017_h_
#define __PoseHypotheses2017_h_

#include "PoseHypothesis2017.h"

#include <vector>

class PoseHypotheses2017
{
public:
  typedef PoseHypothesis2017* element_type;
  typedef std::vector<element_type>::iterator iterator;
  typedef std::vector<element_type>::const_iterator const_iterator;
  typedef std::vector<element_type>::reverse_iterator reverse_iterator;
  typedef std::vector<element_type>::const_reverse_iterator const_reverse_iterator;
  typedef std::vector<element_type>::reference reference;
  typedef std::vector<element_type>::const_reference const_reference;
  typedef std::vector<element_type>::size_type size_type;

private:
  std::vector<element_type> hypotheses;

public:
  ~PoseHypotheses2017()
  {
    clear();
  }

  iterator begin() { return hypotheses.begin(); }
  const_iterator begin() const { return hypotheses.begin(); }

  reverse_iterator rbegin() { return hypotheses.rbegin(); }
  const_reverse_iterator rbegin() const { return hypotheses.rbegin(); }

  iterator end() { return hypotheses.end(); }
  const_iterator end() const { return hypotheses.end(); }

  reverse_iterator rend() { return hypotheses.rend(); }
  const_reverse_iterator rend() const { return hypotheses.rend(); }

  bool empty() const { return hypotheses.empty(); }
  void clear()
  {
    // delete and remove all hypothesis
    while (!empty())
    {
      pop_back();
    }
  }

  iterator erase(const iterator &it)
  {
    delete *it;
    return hypotheses.erase(it);
  }

  void push_back(const Pose2f &newPose, float positionConfidence, float symmetryConfidence, const unsigned &timestamp, const SelfLocator2017Parameters &parameters)
  {
    hypotheses.push_back(new PoseHypothesis2017(newPose, positionConfidence, symmetryConfidence, timestamp, parameters));
  }
  void push_back(const PoseHypothesis2017 &hypothesis, const unsigned &timeStamp)
  {
    hypotheses.push_back(new PoseHypothesis2017(hypothesis, timeStamp));
  }
  void pop_back()
  {
    delete hypotheses[hypotheses.size() - 1];
    hypotheses.pop_back();
  }

  size_type size() const { return hypotheses.size(); }

  reference operator[](size_type idx) { return hypotheses[idx]; }
  const_reference operator[](size_type idx) const { return hypotheses[idx]; }

  reference front() { return hypotheses.front(); }
  const_reference front() const { return hypotheses.front(); }

  reference back() { return hypotheses.back(); }
  const_reference back() const { return hypotheses.back(); }

  reference at(size_type idx) { return hypotheses.at(idx); }
  const_reference at(size_type idx) const { return hypotheses.at(idx); }
};

#endif //__PoseHypotheses2017_h_
