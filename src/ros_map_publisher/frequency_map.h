#pragma once

#include <Eigen/Core>

#include "g2o/types/data/raw_laser.h"

class FrequencyMapCell {
 public:
  FrequencyMapCell(int hits_ = 0, int misses_ = 0);
  virtual ~FrequencyMapCell() {}

  int hits() { return _hits; }
  void setHits(int hits_) { _hits = hits_; }

  int misses() { return _misses; }
  void setMisses(int misses_) { _misses = misses_; }

  void incrementHits(int x) { _hits += x; }
  void decrementHits(int x) { _hits -= x; }
    
  void incrementMisses(int x) { _misses += x; }
  void decrementMisses(int x) { _misses -= x; }

 protected:
  int _hits; 
  int _misses;
};

class FrequencyMap : public Eigen::Matrix<FrequencyMapCell, Eigen::Dynamic, Eigen::Dynamic> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  FrequencyMap();
  FrequencyMap(float resolution_, Eigen::Vector2f &offset_, Eigen::Vector2i &size, FrequencyMapCell &unknown);
  virtual ~FrequencyMap() {}

  void integrateScan(const g2o::RawLaser* laser, const g2o::SE2 &robotPose, 
		     float maxRange = -1.0f, float usableRange = -1.0f, float infinityFillingRange = -1.0f, int gain = 1, int squareSize = 1);
  void applyGain(int gain);

  inline Eigen::Vector2i world2map(const Eigen::Vector2f &wp) const { 
    return Eigen::Vector2i(lrint((wp.x() - _offset.x()) / _resolution),
			   lrint((wp.y() - _offset.y()) / _resolution)); 
  }

  
  bool isInside(const Eigen::Vector2i &mp) const { 
    return mp.x() >= 0 && mp.y() >=0 && mp.x() < this->rows() && mp.y() < this->cols();
}

 protected:
  Eigen::Vector2f _offset;
  float _resolution;  
};
