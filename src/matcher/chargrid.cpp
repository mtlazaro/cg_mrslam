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

#include "chargrid.h"
#include <omp.h>
using namespace std;
using namespace Eigen;

/* helper functios begin */

void CharGrid::addToPrunedMap(std::map<DiscreteTriplet, MatcherResult>& myMap, MatcherResult& mr, double dx, double dy, double dth){
  DiscreteTriplet currentTriplet(mr.transformation, dx, dy, dth);
  std::map<DiscreteTriplet, MatcherResult>::iterator it=myMap.find(currentTriplet);
  if (it!=myMap.end()){
    // another triplet is found;
    if (it->second.score>mr.score)
      it->second = mr;
  } else {
    myMap.insert(std::make_pair(currentTriplet, mr));
  }
};

inline int _l2(int k)
{
  int i=0;
  while(k>0)
  {
    k=k>>1;
    i++;
  }
  return i;
}

/* helper functions end */

struct PointAccumulator {
  PointAccumulator(){
    _acc.setZero();
    _count = 0;
  }
  int count() const {return _count;}
  Vector2d mean() const { 
    if ( _count ) 
      return _acc * (1. / (double) _count);
    return _acc;
  }

  void add(const Vector2d &p) {
    _acc += p;
    _count ++;
  }
  void remove(const Vector2d& p) {
    if (_count==0)
      return;
    _acc -= p;
    _count --;
  }

  Vector2d  _acc;
  int _count;
};

struct Vector2iComparator {
  bool operator () (const Vector2i& v1, const Vector2i& v2){
    if (v1.x() < v2.x() || (v1.x() == v2.x() && v1.y() < v2.y()))
      return true;
    return false;
  }
};
//typedef  std::map<Vector2i, PointAccumulator, Vector2iComparator> Vector2iAccumulatorMap;
typedef  std::map<Vector2i, PointAccumulator, Vector2iComparator, Eigen::aligned_allocator<std::pair<Vector2i, PointAccumulator> > > Vector2iAccumulatorMap;

void CharGrid::subsample(Vector2dVector& dest, const Vector2dVector & src, double res){

  double ires = 1./res;
  Vector2iAccumulatorMap accMap;
  for (Vector2dVector::const_iterator it = src.begin(); it!=src.end(); it++){
    const Vector2d& p = *it;
    Vector2i ip(ires * p.x(), ires * p.y());
    Vector2iAccumulatorMap::iterator ait = accMap.find(ip);
    if (ait==accMap.end()){
      PointAccumulator pa;
      pa.add(p);
      accMap.insert(make_pair(ip, pa));
    } else {
      PointAccumulator& pa = ait->second;
      pa.add(p);
    }
  }

  dest.resize(accMap.size());
  int i = 0;
  for (Vector2iAccumulatorMap::iterator ait=accMap.begin(); ait!=accMap.end(); ait++){
    dest[i] = ait->second.mean();
    i++;
  }
}

CharGrid::CharGrid(Vector2f lowerLeft_, Vector2f upperRight_, float res_, int kscale_)
{
  _kscale = kscale_;
  _grid=CharGridMap(lowerLeft_, upperRight_, res_);
}

CharGrid::~CharGrid(){;}

void CharGrid::applyKernel(CharGridMap& out, const MatrixXChar& kernel, int r, int c)
{
  const unsigned char* ker = kernel.data();
  int kRows = kernel.rows();
  int kCols = kernel.cols();
 
  Vector2i outsize = out.size();

  int oRows = outsize.x();
  int oCols = outsize.y();
  //compute the center of the kernel
  int center = (kRows-1)/2;
  for (int i = 0; i < kRows; i++)
  {
    int iOut = r+i-center;
    if((iOut >= 0) && (iOut <oRows))
    {
      for(int j = 0; j < kCols; j++)
      {
	int jOut = c+j-center;
	if((jOut >= 0) && (jOut < oCols))
	{
	  unsigned char& v = out.cell(iOut,jOut);
	  const unsigned char& k = ker[j*kRows+i];
	  v = (k<v) ? k : v;
	}
      }
    }
  }
}

double CharGrid::greedySearch(Vector3d& result, const Vector2dVector& points, 
			  Vector3f lowerLeftF, Vector3f upperRightF, 
			  double thetaRes, double maxScore) {
  std::vector<MatcherResult> mresvec;
  double dx = grid().resolution()*4;
  double dy = grid().resolution()*4;
  double dth = thetaRes *4;
  
  greedySearch(mresvec, points, lowerLeftF, upperRightF, thetaRes,
		    maxScore, dx, dy, dth);
  
  if (mresvec.size()){
    result = mresvec[0].transformation;
    return mresvec[0].score;
  }
  return std::numeric_limits<double>::max();
}


void CharGrid::greedySearch(std::vector<MatcherResult>& mresvec, 
		    const Vector2dVector& points, 
		    Eigen::Vector3f lowerLeftF, Eigen::Vector3f upperRightF, double thetaRes,
			double maxScore, double dx, double dy, double dth){
  RegionVector regions(1);
  regions[0].lowerLeft=lowerLeftF;
  regions[0].upperRight=upperRightF;
  MatchingParameters params;
  params.searchStep=Vector3d(grid().resolution(), grid().resolution(), thetaRes);
  params.maxScore=maxScore;
  params.resultsDiscretization=Vector3d(dx,dy,dth);
  return greedySearch(mresvec, points, regions, params);
}

void CharGrid::greedySearch(std::vector<MatcherResult>& mresvec, 
			const Vector2dVector& points, 
		    	const RegionVector& regions, 
			double thetaRes, double maxScore, double dx, double dy, double dth){

  MatchingParameters params;
  params.searchStep=Vector3d(grid().resolution(), grid().resolution(), thetaRes);
  params.maxScore=maxScore;
  params.resultsDiscretization=Vector3d(dx,dy,dth);
  return greedySearch(mresvec, points, regions, params);
}

void CharGrid::greedySearch(std::vector<MatcherResult>& mresvec, 
			const Vector2dVector& points,
			const RegionVector& regions, const MatchingParameters& params)
			    
{
  double thetaRes = params.searchStep.z();
  int xSteps = params.searchStep.x()/_grid.resolution();
  int ySteps = params.searchStep.y()/_grid.resolution();
  double maxScore = params.maxScore;

  if (xSteps <=0)
    xSteps = 1;
  if (ySteps <=0)
    ySteps = 1;

  size_t maxNumThreads = 4;
  size_t numThreads = (regions.size() < maxNumThreads) ? regions.size() : maxNumThreads;
  size_t regionChunkSize=regions.size()/numThreads;
  std::map<DiscreteTriplet,MatcherResult> resultMap[numThreads];
  #pragma omp parallel num_threads(numThreads) 
  {
    Vector2iVector intPoints(points.size());
    size_t threadId = omp_get_thread_num();
    size_t imin=threadId*regionChunkSize;
    size_t imax = (threadId==numThreads-1) ? regions.size() : (threadId+1)*regionChunkSize; 
    for(size_t reg = imin; reg<imax; reg++){
      const Region& region = regions[reg];
      Vector3f lowerLeftF = region.lowerLeft;
      Vector3f upperRightF = region.upperRight;
      Vector2i lowerLeft  = _grid.world2grid(Vector2f(lowerLeftF.x(),  lowerLeftF.y()));
      Vector2i upperRight = _grid.world2grid(Vector2f(upperRightF.x(), upperRightF.y()));
      for(double t=lowerLeftF.z(); t<upperRightF.z(); t+=thetaRes)
	{
	  double c=cos(t), s=sin(t);
	  Vector2i previousPoint(-10000,-10000);
	  int k=0;
	  const Vector2d* _p=&(points[0]);
	  Vector2i* _ip=&(intPoints[0]);
	  for(size_t i = 0; i < points.size(); i++)
	    {
	      Vector2d p(c*_p->x()-s*_p->y(), s*_p->x()+c*_p->y());
	      Vector2i ip(p.x()* _grid.inverseResolution(), p.y()* _grid.inverseResolution());
	      if(ip.x() != previousPoint.x() || ip.y() != previousPoint.y())
		{
		  *_ip=ip;
		  _ip++;
		  k++;
		  previousPoint=ip;
		}
	      _p++;
	    }
    
	  float ikscale = 1./(float)(_kscale);
	  for(int i=lowerLeft.x(); i<upperRight.x(); i+=xSteps)
	    {
	      for(int j=lowerLeft.y(); j<upperRight.y(); j+=ySteps)
		{
		  int idsum=0;
		  Vector2i offset(i,j);
		  const Vector2i* _ip=&(intPoints[0]);
		  for (int ii=0; ii<k; ii++)
		    {
		      Vector2i ip=*_ip+offset;
		      _ip++;
		      idsum+=_grid.cell(ip);
		    }
		  float dsum = (float)idsum * (float)ikscale;
		  dsum = k ? (dsum / (double) k) : maxScore+1;
		  Vector2f mp(_grid.grid2world(Vector2i(i,j)));
		  Vector3d current(mp.x(), mp.y(), t);
		  if (dsum<maxScore){
		    MatcherResult mr(current, dsum);
		    addToPrunedMap(resultMap[threadId],mr,
				   params.resultsDiscretization.x(),
				   params.resultsDiscretization.y(),
				   params.resultsDiscretization.z());
		  }
		}
	    }
	}
    }
  }
  
  int resultsSize=0;
  for (size_t i=0; i<numThreads; i++)
    resultsSize+=resultMap[i].size();
  mresvec.resize(resultsSize,MatcherResult(Vector3d(0.,0.,0.),0));

  
  size_t k=0;
  for (size_t i=0; i<numThreads; i++) {
    for (std::map<DiscreteTriplet,MatcherResult>::iterator it=resultMap[i].begin(); it!=resultMap[i].end(); it++){
      mresvec[k++] = it -> second;
    }									
  }
  //cerr << "Number of results= " << mresvec.size() << endl;

  MatcherResultScoreComparator comp;
  std::sort(mresvec.begin(), mresvec.end(), comp);
}

void CharGrid::hierarchicalSearch(std::vector<MatcherResult>& mresvec, 
			const Vector2dVector& points,
			const RegionVector& regions, const MatchingParametersVector& paramsVec) {
  mresvec.clear();
  RegionVector currentRegions = regions;
  for (size_t i = 0; i<paramsVec.size()-1; i++){
    //cerr << "Iteration " << i << endl;
    MatchingParameters params = paramsVec[i];
    greedySearch(mresvec, points, currentRegions, params);
    if (mresvec.size()){
      //cerr <<  "bestScore iteration " << i << " = " << mresvec[0].score << endl;
      currentRegions.clear();
      for (size_t i = 0; i < mresvec.size(); i++){
	Vector3d best=mresvec[i].transformation;
	Region reg;
	Vector3d newlower = - (params.resultsDiscretization * .5) + best;
	Vector3d newupper =   (params.resultsDiscretization * .5) + best;

	reg.lowerLeft  = Vector3f (newlower.x(), newlower.y(), newlower.z());
	reg.upperRight = Vector3f (newupper.x(), newupper.y(), newupper.z());
	currentRegions.push_back(reg);
      }
    }else
      break;
  }

  if (mresvec.size()){
    //cerr << "Iteration " << paramsVec.size()-1 << endl;
    MatchingParameters params = paramsVec[paramsVec.size()-1];
    greedySearch(mresvec, points, currentRegions, params);
    //if (mresvec.size()){
    //  cerr <<  "bestScore iteration " <<  paramsVec.size()-1 << " = " << mresvec[0].score << endl;
    //}
  }
}

void CharGrid::hierarchicalSearch(std::vector<MatcherResult>& mresvec, 
			      const Vector2dVector& points,
			      Eigen::Vector3f lowerLeftF, Eigen::Vector3f upperRightF, double thetaRes,
			      double maxScore, double dx, double dy, double dth)
			    
{
  RegionVector regions(1);
  regions[0].lowerLeft=lowerLeftF;
  regions[0].upperRight=upperRightF;

  MatchingParametersVector pvec(4);

  pvec[0].searchStep=Vector3d(8*_grid.resolution(), 8*_grid.resolution(), 4*thetaRes);
  pvec[0].maxScore=maxScore;
  pvec[0].resultsDiscretization=Vector3d(dx*8,dy*8,dth*8);

  pvec[1].searchStep=Vector3d(4*_grid.resolution(), 4*_grid.resolution(), 2*thetaRes);
  pvec[1].maxScore=maxScore;
  pvec[1].resultsDiscretization=Vector3d(dx*4,dy*4,dth*4);

  pvec[2].searchStep=Vector3d(2*_grid.resolution(), 2*_grid.resolution(), thetaRes);
  pvec[2].maxScore=maxScore;
  pvec[2].resultsDiscretization=Vector3d(dx*2,dy*2,dth*2);

  pvec[3].searchStep=Vector3d(_grid.resolution(), _grid.resolution(), thetaRes);
  pvec[3].maxScore=maxScore;
  pvec[3].resultsDiscretization=Vector3d(dx,dy,dth);

  hierarchicalSearch(mresvec,  points, regions, pvec);
}
void CharGrid::hierarchicalSearch(std::vector<MatcherResult>& mresvec, 
			const Vector2dVector& points, 
		    	const RegionVector& regions, 
			      double thetaRes, double maxScore, double dx, double dy, double dth, int nLevels){

  MatchingParametersVector pvec;

  for (int i = nLevels-1; i >= 0; i--){
    int m = pow(2, i);
    int mtheta = (m/2 < 1) ? m : m/2; 

    MatchingParameters mp;
    mp.searchStep=Vector3d(m*_grid.resolution(), m*_grid.resolution(), mtheta*thetaRes);
    mp.maxScore=maxScore;
    mp.resultsDiscretization=Vector3d(dx*m,dy*m,dth*m);
    
    pvec.push_back(mp);
  }



  return hierarchicalSearch(mresvec, points, regions, pvec);


}

void CharGrid::hierarchicalSearch(std::vector<MatcherResult>& mresvec, 
			      const Vector2dVector& points,
			      Eigen::Vector3f lowerLeftF, Eigen::Vector3f upperRightF, double thetaRes,
			      double maxScore, double dx, double dy, double dth, int nLevels)
			    
{
  RegionVector regions(1);
  regions[0].lowerLeft=lowerLeftF;
  regions[0].upperRight=upperRightF;

  return hierarchicalSearch(mresvec,  points, regions, thetaRes, maxScore, dx, dy, dth, nLevels);
}



void CharGrid::countPoints(Eigen::Vector2f lowerLeftF, Eigen::Vector2f upperRightF,
				double *score)
			    
{
  
  Vector2i lowerLeft  = _grid.world2grid(lowerLeftF);
  Vector2i upperRight = _grid.world2grid(upperRightF);
    
  int isum = 0;
  for(int i=lowerLeft.x(); i<upperRight.x(); i++){
      for(int j=lowerLeft.y(); j<upperRight.y(); j++){
	Vector2i offset(i,j);
	isum+=_grid.cell(offset);
      }
  }
  
  int visitedCells = (upperRight.x() - lowerLeft.x()) * (upperRight.y() - lowerLeft.y());
  
  //std::cerr << "isum = " << isum << std::endl;
  //std::cerr << "visitedCells = " << visitedCells << std::endl;
 
  *score = (float)isum/(float)visitedCells;
  
}


void CharGrid::searchNonMatchedPoints(const Vector2dVector& points, Vector2dVector& nonmatchedpoints, double maxScore){
  nonmatchedpoints.clear();
  float ikscale = 1./(float)(_kscale);
  for(size_t i = 0; i < points.size(); i++){
    Vector2i gpoint  = _grid.world2grid(Vector2f(points[i].x(), points[i].y()));
    double value = (float) _grid.cell(gpoint) * ikscale;
    if (value > maxScore)
      nonmatchedpoints.push_back(points[i]);
  }
}
