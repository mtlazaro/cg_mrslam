#ifndef GRIDMAP_HH
#define GRIDMAP_HH

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include "array_allocator.h"


/**Generic grid map class.
Supports copy constuction, assignment, resize, and indicidual cell access.*/
template <class T>
struct _GridMap
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef T CellType;


  /**Mapping function between world coordinates and map cells.
  @param wp: world point
  @returns the indices of the cell corresponding to wp in the grid
  */
  inline Eigen::Vector2i world2grid(const Eigen::Vector2f& wp) const
  {
    return Eigen::Vector2i(lrint((wp.x()-_lowerLeft.x())*_inverseResolution), lrint((wp.y()-_lowerLeft.y())*_inverseResolution));
  }

  
  /**Mapping function between world coordinates and map cells.
  @param x: x coordinate
  @param y: y coordinate
  @returns the indices of the cell corresponding to wp in the grid
  */
  inline Eigen::Vector2i world2grid(const float x, const float y) const
  {
    return Eigen::Vector2i(lrint((x -_lowerLeft.x())*_inverseResolution), lrint((y -_lowerLeft.y())*_inverseResolution));
  }
  

  /**Mapping function between map cells and world coordinates.
  @param gp: grid cell
  @returns the coordinates of the cell mp in world coordinates
  */
  Eigen::Vector2f grid2world(const Eigen::Vector2i& gp) const
  {
    return Eigen::Vector2f((float) _lowerLeft.x()+(_resolution*(float)gp.x()), (float) _lowerLeft.y()+(_resolution*(float)gp.y()));
  }


  /**Boundary check
  @param mp: the cell to be checked
  @returns true if a cell is in the map, false otherwise
  */
  bool isInside(const Eigen::Vector2i& mp) const
  {
    return mp.x()>=0 && mp.y()>=0 && mp.x()<size().x() && mp.y()<size().y();
  }

  
  /**Cell accessor method. Returns the cell located at the indices passed as argument.
  @param x: the row index
  @param y: the column index
  @returns the cell at indices p in the map
  */
  inline T& cell(const int& x, const int& y)
  {
    return _allocator[x][y];
  }


  /**Const cell accessor method. Returns the cell located at the indices passed as argument.
  @param x: the row index
  @param y: the column index
  @returns the cell at indices p in the map
  */
  inline const T& cell(const int& x, const int& y) const
  {
    return _allocator[x][y];
  }


  /**Cell accessor method. Returns the cell located at the indices passed as argument.
  @param p: the cell indices
  @returns the cell at indices p in the map
  */
  inline T& cell(const Eigen::Vector2i& p)
  {
    return _allocator[p.x()][p.y()];
  }


  /**Const cell accessor method. Returns the cell located at the indices passed as argument.
  @param p: the cell indices
  @returns the cell at indices p in the map
  */
  inline const T& cell(const Eigen::Vector2i& p) const
  {
    return _allocator[p.x()][p.y()];
  }

  
  /**Center accessor method. Returns the indices of the center */
  inline Eigen::Vector2f center() const
  {
    return .5 * (_upperRight + _lowerLeft);
  }

  /**Set a new center for the gridmap, translating upperRight and lowerLeft corners
  @param newCenter: the new center we want to set the grid's center to
  */
  inline void setCenter(const Eigen::Vector2f& newCenter)
  {
    Eigen::Vector2f t = newCenter - (.5 * (_upperRight + _lowerLeft));
    _upperRight += t;
    _lowerLeft  += t;
  }

  
  /**Set a new center for the gridmap, translating upperRight and lowerLeft corners
  @param newCenterX: X coordinate of the new center
  @param newCenterY: Y coordinate of the new center
  */
  inline void setCenter(const float& x, const float& y)
  {
    Eigen::Vector2f t(x - (.5 * (_upperRight.x() + _lowerLeft.x())), y - (.5 * (_upperRight.y() + _lowerLeft.y())));
    _upperRight += t;
    _lowerLeft  += t;
  }
  
  
  inline float resolution() const {return _resolution;}
  inline float inverseResolution() const {return _inverseResolution;}
  inline Eigen::Vector2i size()     const { Eigen::Vector2i v; v[0]=_allocator.rows(), v[1]=_allocator.cols(); return v;}
  inline const Eigen::Vector2f& lowerLeft() const {return _lowerLeft;}
  inline const Eigen::Vector2f& upperRight() const {return _upperRight;}
  inline Eigen::Vector2f lowerLeft() {return _lowerLeft;}
  inline Eigen::Vector2f upperRight() {return _upperRight;}


  /**Constructs an empty gridmap.*/
  _GridMap();


  /**Constructs a map of a given size, offset and resolution.
  The map is filled with unknown cells.
  @param size: the size in cells
  @param offset: the location of the cell 0,0 in world coordinates
  @param resolution: the resolution of the map
  @param unknownCell: the cell value to be used for filling the map
  */
  _GridMap(const Eigen::Vector2i& size, const Eigen::Vector2f& offset, const float& resolution, const T& unknownCell=T());


  /**Constructs a map of a given size, through lowerLeft and upperRight corners, offset and resolution.
  The map is filled with unknown cells.
  @param lowerLeft: the location of the cell 0,0 in world coordinates
  @param upperRight: up-right corner of the map
  @param resolution: the resolution of the map
  @param unknownCell: the cell value to be used for filling the map
  */
  _GridMap(const Eigen::Vector2f& lowerLeft, const Eigen::Vector2f& upperRight, const float& resolution, const T& unknownCell=T());


  /**Resize operator
  It resizes the map so that the minimum represented world value will be in min and the maximum in max.
  Uninitialized cells will be padded with unknownval.
  @param min: the lower left corner in world coordinates
  @param max: the upper right corner in world coordinates
  @param unknownCell: the value to be used for padding new cells
  */
  _GridMap<T> resize(const Eigen::Vector2f min, Eigen::Vector2f max, T& unknownCell);


  bool load(std::istream& is);
  void save(std::ostream& os) const;
  void saveAsPPM(std::ostream& os, bool eq) const;


protected:
  /**resolution and inverse resolution of the grid map*/
  float _resolution;
  float _inverseResolution;

  Eigen::Vector2f _lowerLeft;
  Eigen::Vector2f _upperRight;
  
  _Array2DAllocator<0,0,CellType> _allocator;
};


template <class T>
_GridMap<T>::_GridMap(): _allocator(0,0) {}


template <class T>
_GridMap<T>::_GridMap(const Eigen::Vector2f& lowerLeft, const Eigen::Vector2f& upperRight, const float& res, const T& unknownCell)
{
  _lowerLeft = lowerLeft;
  _upperRight = upperRight;
  _resolution = res;
  _inverseResolution = 1./_resolution;
  Eigen::Vector2f dSize = (upperRight - lowerLeft) * _inverseResolution;
  Eigen::Vector2i iSize(dSize.x(), dSize.y());
  _allocator = _Array2DAllocator<0,0,CellType>(iSize.x(), iSize.y());
  
  for(int i = 0; i < size().x(); ++i)
  {
    for(int j = 0; j < size().y(); ++j)
    {
      _allocator[i][j] = unknownCell;
    }
  }
}


template <class T>
_GridMap<T>::_GridMap(const Eigen::Vector2i& s, const Eigen::Vector2f& off, const float& res, const T& unknownCell): _allocator(s.x(), s.y())
{
  _allocator = _Array2DAllocator<0,0,CellType>(s.x(), s.y());
  _resolution = res;
  _inverseResolution = 1./_resolution;
  _lowerLeft = off;
  _upperRight = Eigen::Vector2f(s.x(), s.y()) + _lowerLeft;

  for(int i = 0; i < size().x(); ++i)
  {
    for(int j=0; j < size().y(); ++j)
    {
      _allocator[i][j] = unknownCell;
    }
  }
}


template <class T>
_GridMap<T> _GridMap<T>::resize(const Eigen::Vector2f min, Eigen::Vector2f max, T& unknownCell)
{
  Eigen::Vector2i newSize((int) ((max.x()-min.x())/resolution()), (int) ((max.y()-min.y())/resolution()));
  _GridMap<T> newMap(newSize, resolution(), min, unknownCell);
  for(int i = 0; i < newSize.x(); ++i)
  {
    for(int j = 0; j < newSize.y(); ++j)
    {
      Eigen::Vector2i c = world2grid(newMap.grid2world(Eigen::Vector2i(i,j)));
      if(isInside(c))
      {
	newMap._allocator[i][j] = _allocator[c.x()][c.y()];
      }
    }
  }
  return newMap;
}


template <class T>
bool _GridMap<T>::load(std::istream& is)
{
  std::string tag;
  while (is && tag!="#GRIDMAP")
    is >> tag;
  if (tag!="#GRIDMAP")
    return false;
  is >> tag;
  if (tag!="#SIZE")
    return false;
  Eigen::Vector2i s;
  is >> s.x() >> s.y();
  is >> tag;
  if (tag!="#RESOLUTION")
    return false;
  float res;
  is >> res;
  is >> tag;
  if (tag!="#OFFSET")
    return false;
  Eigen::Vector2f off;
  is >> off.x() >> off.y();
  T empty=T();
  *this=_GridMap<T>(s,res,off,empty);
  is >> tag;
  if (tag!="#CELLDATA_START")
    return false;
  for (int i=size().y()-1; i>=0; i--)
  {
    for (int j=0; j<size().x(); j++)
    {
      is >> cell(Eigen::Vector2i(j,i));
    }
  }
  is >> tag;
  if (tag!="#CELLDATA_END")
    return false;
  return true;
}


template <class T>
void _GridMap<T>::save(std::ostream& os) const
{
  os << "#GRIDMAP" << std::endl;
  os << "#SIZE " << size().x() << " " << size().y() << std::endl;
  os << "#RESOLUTION " << resolution() << std::endl;
  os << "#OFFSET " << lowerLeft().x() << " " << lowerLeft().y() << std::endl;
  os << "#CELLDATA_START" << std::endl;
  for (int i=size().y()-1; i>=0; i--)
  {
    for (int j=0; j<size().x(); j++)
    {
      os << cell(Eigen::Vector2i(j,i)) << " ";
    }
    os << std::endl;
  }
  os << "#CELLDATA_END" << std::endl;
}


template <class T>
void _GridMap<T>::saveAsPPM(std::ostream& os, bool equalize) const
{
  os << "P6" << std::endl;
  os << "#resolution " << resolution() << std::endl;
  os << "#offset "     << lowerLeft().x() << " " << upperRight().y() << std::endl;
  os << size().x() << " " << size().y() << std::endl  << 255 << std::endl;

  int height = this->size().y();
  int width = this->size().x();
  float max=1.;
  if(equalize)
  {
    for(int i = 1; i <= height; ++i)
    {
      for(int x = 0; x < width; ++x)
      {
	int y = height-i;
	float k = cell(Eigen::Vector2i(x,y));
	if(k > max)
	  max = k;
      }
      if(max != 0)
      {
	max=1./max;
      }
      else
      {
	max=1.;
      }
    }
  }

  for(int i = 1; i <= height; ++i)
  {
    for(int x = 0; x < width; ++x)
    {
      int y = height-i;
      float fo = cell(Eigen::Vector2i(x,y));
      float occ = fo*max;
      unsigned char c = (unsigned char)(255.-255*occ);
      unsigned char r=c, g=c, b=c;
      if(fo==-1.)
      {
	b=(unsigned char)(210);
	g=(unsigned char)(190);
	r=(unsigned char)(190);
      }
      else if(fo==-2.)
      {
	b=(unsigned char)(64);
	g=(unsigned char)(64);
	r=(unsigned char)(255);
      }
      else if (fo==-3.)
      {
	b=(unsigned char)(255);
	g=(unsigned char)(64);
	r=(unsigned char)(64);
      }
      os.put(r);
      os.put(g);
      os.put(b);
    }
  }
};
#endif
