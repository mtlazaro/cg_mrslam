// Copyright (c) 2013, Maria Teresa Lazaro Grañon
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this
//   list of conditions and the following disclaimer in the documentation and/or
//   other materials provided with the distribution.
//
//   Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef _CHARGRID_MAP_H_
#define _CHARGRID_MAP_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <limits>
#include <vector>
#include <map>

#include "gridmap.h"


typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > Vector2iVector;
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > Vector2dVector;
typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> MatrixXChar;
typedef _GridMap<unsigned char> CharGridMap;


struct MatcherResult {
  MatcherResult(const Eigen::Vector3d& transformation, const double& score, 
		const Eigen::Matrix3d& informationMatrix = Eigen::Matrix3d::Identity()){
    this->transformation = transformation;
    this->score = score;
    this->informationMatrix = informationMatrix;
  }
  Eigen::Vector3d transformation;
  double score;
  Eigen::Matrix3d informationMatrix;
};

struct MatcherResultScoreComparator {
  bool operator() (const MatcherResult& mr1, const MatcherResult& mr2){
    return mr1.score<mr2.score;
  } 
};

struct DiscreteTriplet {
  DiscreteTriplet(Eigen::Vector3d& tr, double dx, double dy, double dth) {
    ix=(int)(tr.x()/dx);
    iy=(int)(tr.y()/dy);
    ith = (int)(tr.z()/dth);
  }

  bool operator < (const DiscreteTriplet& dt) const {
    if (ix<dt.ix )
      return true;
    if (ix == dt.ix && iy<dt.iy) 
      return true;
    if (ix == dt.ix && iy == dt.iy && ith < dt.ith)
      return true;
    return false;
  }
  double ix, iy, ith;
};

struct Region{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Eigen::Vector3f lowerLeft;
  Eigen::Vector3f upperRight;
};
typedef std::vector<Region, Eigen::aligned_allocator<Region> > RegionVector;

struct MatchingParameters{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Eigen::Vector3d searchStep;
  Eigen::Vector3d resultsDiscretization;
  double maxScore;
};


typedef std::vector<MatchingParameters, Eigen::aligned_allocator<MatchingParameters> > MatchingParametersVector;



struct CharGrid
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  CharGrid(Eigen::Vector2f lowerLeft_, Eigen::Vector2f upperRight_, float res_, int _kscale = 128);
  ~CharGrid();

  static void addToPrunedMap(std::map<DiscreteTriplet, MatcherResult>& myMap, MatcherResult& mr, double dx, double dy, double dth);
  static void subsample(Vector2dVector& dest, const Vector2dVector & src, double res);

  inline const CharGridMap& grid() const {return _grid;}
  inline CharGridMap& grid()             {return _grid;}

  //! search for the best in a region
  //! @param result: the returned transformation
  //! @param points: the input scan points in cartesian coordinates
  //! @param lowerLeftF: the lower left corner of the search region (z is the theta component in radiants)
  //! @param upperRightF: the upper right element of the search region (z is the theta component in radiants)
  //! @param thetaRes: the resolution of the search
  //! @param maxScore: the maximum score of the match
  //! @returns the score of the best transformation
  double greedySearch(Eigen::Vector3d& result, const Vector2dVector& points, 
		      Eigen::Vector3f lowerLeftF, Eigen::Vector3f upperRightF, 
		      double thetaRes, double maxScore);

  // search for the N best in a region
  //! @param mresvec: the returned vector of matche results
  //! @param points: the input scan points in cartesian coordinates
  //! @param lowerLeftF: the lower left corner of the search region (z is the theta component in radiants)
  //! @param upperRightF: the upper right element of the search region (z is the theta component in radiants)
  //! @param thetaRes: the resolution of the search
  //! @param maxScore: the maximum score of the match
  void greedySearch(std::vector<MatcherResult>& mresvec, 
		    const Vector2dVector& points, 
		    Eigen::Vector3f lowerLeftF, Eigen::Vector3f upperRightF, double thetaRes,
		    double maxScore, double dx, double dy, double dth);
 
  // search for the N best in a set of regions
  //! @param mresvec: the returned vector of matche results
  //! @param points: the input scan points in cartesian coordinates
  //! @param regions: the regions where to do the search (vector)
  //! @param thetaRes: the resolution of the search
  //! @param maxScore: the maximum score of the match
  void greedySearch(std::vector<MatcherResult>& mresvec, 
		    const Vector2dVector& points, 
		    const RegionVector& regions, 
		    double thetaRes, double maxScore, double dx, double dy, double dth);
  

  // search for the N best in a set of regions region
  //! @param mresvec: the returned vector of matche results
  //! @param points: the input scan points in cartesian coordinates
  //! @param regions: the regions where to do the search (vector)
  //! @param params: the parameters to be used for the matching
  void greedySearch(std::vector<MatcherResult>& mresvec, 
		    const Vector2dVector& points, 
		    const RegionVector& regions, 
		    const MatchingParameters& params);

  
  void hierarchicalSearch(std::vector<MatcherResult>& mresvec, 
			  const Vector2dVector& points,
			  const RegionVector& regions, 
			  const MatchingParametersVector& paramsVec);


  void hierarchicalSearch(std::vector<MatcherResult>& mresvec, 
			  const Vector2dVector& points,
			  Eigen::Vector3f lowerLeftF, Eigen::Vector3f upperRightF, double thetaRes,
			  double maxScore, double dx, double dy, double dth);
  
  void hierarchicalSearch(std::vector<MatcherResult>& mresvec, 
			  const Vector2dVector& points, 
			  const RegionVector& regions, 
			  double thetaRes, double maxScore, double dx, double dy, double dth, int nLevels);
    
  void hierarchicalSearch(std::vector<MatcherResult>& mresvec, 
			      const Vector2dVector& points,
			      Eigen::Vector3f lowerLeftF, Eigen::Vector3f upperRightF, double thetaRes,
			  double maxScore, double dx, double dy, double dth, int nLevels);
  
  void countPoints(Eigen::Vector2f lowerLeftF, Eigen::Vector2f upperRightF,
				  double *score);
  void searchNonMatchedPoints(const Vector2dVector& points, Vector2dVector& nonmatchedpoints, double maxScore);
  void applyKernel(CharGridMap& out, const MatrixXChar& kernel, int r, int c);
  
  template <typename T>
  void addPoints(typename T::const_iterator begin_, typename T::const_iterator end_, const float& val=0.)
  {
    for(typename T::const_iterator it=begin_; it!=end_; ++it)
      {
	Eigen::Vector2d myP=*it;
	Eigen::Vector2f p(myP.x(), myP.y());
	//Eigen::Vector2i ip = _grid.world2grid(myP);
	Eigen::Vector2i ip = _grid.world2grid(p);
	_grid.cell(ip)=val;
      }
  }

  template <typename T>
  void addAndConvolvePoints(typename T::const_iterator begin_, typename T::const_iterator end_,  const MatrixXChar& kernel)
  {
    for(typename T::const_iterator it=begin_; it!=end_; ++it)
      {
	Eigen::Vector2d myP=*it;
	Eigen::Vector2f p(myP.x(), myP.y());
	//	Eigen::Vector2i ip=_grid.world2grid(myP);
	Eigen::Vector2i ip=_grid.world2grid(p);
	applyKernel(_grid, kernel, ip.x(), ip.y());
      }
  }

  template <typename T>
  void integrateScan(T& m, typename T::const_iterator begin_, typename T::const_iterator end_, const Eigen::Isometry2d& tsf)
  {
    for(typename T::const_iterator it=begin_; it!=end_; ++it)
      {
	Eigen::Vector2d p=*it;
	p = tsf*p;
	m.push_back(p);
      }
  }
  
  CharGridMap _grid;
  int _kscale;
};
#endif
