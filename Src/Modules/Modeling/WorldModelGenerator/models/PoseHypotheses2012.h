/**
* @file PoseHypothesis2012.h
*
* This class represents a collection of PoseHypothesis.
* It secures that there is no memory leakage.
*
* @author <a href="mailto:dino.menges@tu-dortmund.de">Dino Menges</a>
*/

#ifndef __PoseHypotheses2012_h_
#define __PoseHypotheses2012_h_

#include "PoseHypothesis2012.h"

class PoseHypotheses2012
{
private:
  std::vector<PoseHypothesis2012*> hypotheses;

public:
  typedef std::vector<PoseHypothesis2012*>::iterator iterator;
  typedef std::vector<PoseHypothesis2012*>::const_iterator const_iterator;
  typedef std::vector<PoseHypothesis2012*>::reverse_iterator reverse_iterator;
  typedef std::vector<PoseHypothesis2012*>::const_reverse_iterator const_reverse_iterator;
  typedef std::vector<PoseHypothesis2012*>::reference reference;
  typedef std::vector<PoseHypothesis2012*>::const_reference const_reference;
  typedef std::vector<PoseHypothesis2012*>::size_type size_type;

  iterator begin() { return hypotheses.begin(); }
  const_iterator begin() const { return hypotheses.begin(); }

  reverse_iterator rbegin() { return hypotheses.rbegin(); }
  const_reverse_iterator rbegin() const { return hypotheses.rbegin(); }

  iterator end() { return hypotheses.end(); }
  const_iterator end() const { return hypotheses.end(); }

  reverse_iterator rend() { return hypotheses.rend(); }
  const_reverse_iterator rend() const { return hypotheses.rend(); }

  bool empty() const { return hypotheses.empty(); }
  void clear() { hypotheses.clear(); }

  void insert(iterator position, const iterator &first, const iterator &last) { hypotheses.insert(position, first, last); }
  iterator erase(const iterator &it) { delete *it; return hypotheses.erase(it); }

  void push_back(PoseHypothesis2012* hypothesis) { hypotheses.push_back(hypothesis); }
  void pop_back() { delete hypotheses[hypotheses.size() - 1]; hypotheses.pop_back(); }

  size_type size() const { return hypotheses.size(); }

  reference operator[](size_type idx) { return hypotheses[idx]; }
  const_reference operator[](size_type idx) const { return hypotheses[idx]; }
};

#endif //__PoseHypotheses2012_h_
