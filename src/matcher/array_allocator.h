#ifndef _ARRAY_ALLOCATOR_HH_
#define _ARRAY_ALLOCATOR_HH_
#include <assert.h>
#include <string.h>
#include "macros.h"

/**@addtogroup utils*/
//@{
/**This class implements a memory manager for a fixed size array of elements.
It implements copy constructor, assigment and accessor to the elements via  abracket operator*/
template <int N, typename Base>
  struct _ArrayAllocator{
    typedef Base BaseType;
    static const int TemplateSize=N;
    /**constructs a fized size vector whose type is Base of size N*/
    _ArrayAllocator(int n=N) {
/*       if (n!=N) */
/* 	std::cerr << "Fatal n= " << n << " N=" << N << std::endl; */
      assert (n==N); (void) n;
    }
    /**size of the storage*/
    int size() const {return N;}

    /**pointer to the storage base*/
    Base* ptr() {return _svalues;}

    /**const pointer to the storage base*/
    const Base* ptr() const {return _svalues;}

    /**const accessor to the ith element*/
    inline Base& operator [] (int i) { return _svalues[i]; }

    /**accessor to the ith element*/
    inline const Base& operator [] (int i) const { return _svalues[i]; }
  protected:
    Base _svalues[N];
  };


/**This class implements a memory manager for a dynamically sized array of elements.
It implements copy constructor, assigment and accessor to the elements via  abracket operator.
It is implemented as a specialization of the static type.
It implements copy constructor, destructors and assignment operators, and it can e resized.*/
template <>
template <typename Base>
struct _ArrayAllocator <0, Base>{
  typedef Base BaseType;
  static const int TemplateSize=0;

  /**constructs a fized size vector whose type is Base of size N*/
  _ArrayAllocator(int n=0) {
    allocate(n);
  }
  
  /**destructor, releases memory*/
  ~_ArrayAllocator(){
    deallocate();
  }
  
  /**assignment*/
  _ArrayAllocator<0,Base>& operator = (const _ArrayAllocator <0, Base>& src){
    resize(src.size());
    copy(src._dvalues);
    return *this;
  }
  
  /**copy*/
  _ArrayAllocator(const _ArrayAllocator <0, Base>& src) {
    allocate(src.size());
    copy(src._dvalues);
  }

  /**returns the size*/
  int size() const { return _size;}

  /**pointer to the base*/
  Base* ptr() {return _dvalues;}

  /**const pointer to the base*/
  const Base* ptr() const {return _dvalues;}

  /**accessor*/
  inline Base& operator [] (int i) { return _dvalues[i]; }

  /**const accessor*/
  inline const Base& operator [] (int i) const { return _dvalues[i]; }

  /**resizes the storage*/
  void resize(int s) {
    if (s==_size)
      return;
    if (! s)
      deallocate();
    allocate(s);
  }

 protected:
  void copy(const Base* src){
    if ( AISNAV_IS_POD(Base) )
      memcpy(_dvalues, src, _size*sizeof(Base));
    else
      for (int i=0; i<_size; i++)
	_dvalues[i]=src[i];
  }

  void deallocate() {
    if (_size)
      delete [] _dvalues;
    _dvalues=0;
    _size=0;
  }

  void allocate(int n) {
    _dvalues = n  ? new Base[n] : 0;
    _size=n;
  }
    int _size;
    Base* _dvalues;
  };


/**oonverts a static  storage to a dynamic one*/
template <int N, typename Base>
  void st2dyn (_ArrayAllocator<0,Base>& dest, const _ArrayAllocator<N,Base>& src) {
  dest.resize(src.size());
  for (int i=0; i<src.size(); i++)
    dest[i]=src[i];
}

/**oonverts a dynamic storage  to a static one*/
template <int N, typename Base>
  void dyn2st(_ArrayAllocator<N,Base>& dest, const _ArrayAllocator<0,Base>& src){
  assert (dest.size()==src.size());
  for (int i=0; i<src.size(); i++)
    dest[i]=src[i];
}

/**This class implements a memory manager for a fixed size 2D array of elements.
It implements copy constructor, assigment and accessor to the elements via  abracket operator*/
template <int N, int M, typename Base>
  struct _Array2DAllocator{
    typedef Base BaseType;
    static const int TemplateRows = N;
    static const int TemplateCols = M;
    typedef _Array2DAllocator<M,N,Base> TransposedType;

    /**constructor*/
    _Array2DAllocator(int rows_=N, int cols_=M) {
      assert (rows_==N); (void) rows_;
      assert (cols_==M); (void) cols_;
    }

    /**returns the pointer to the ith row*/
    inline Base* operator[] (int i) {return _svalues[i];}

    /**returns the coinst pointer to the ith row*/
    inline const Base* operator [] (int i) const {return _svalues[i];}

    /**returns the number of rows*/
    int rows() const {return TemplateRows;}

    /**returns the number of columns*/
    int cols() const {return TemplateCols;}
  protected:
    Base _svalues[N][M];
  };

/**This class implements a memory manager for a dynamic size 2D array of elements.
It implements copy constructor, assigment and accessor to the elements via  abracket operator*/
template <>
template <typename Base>
struct _Array2DAllocator<0,0,Base>{
    typedef Base BaseType;
    static const int TemplateRows = 0;
    static const int TemplateCols = 0;
    typedef _Array2DAllocator<0,0,Base> TransposedType;

    /**returns the pointer to the ith row*/
    inline Base* operator[] (int i) {return _dvalues[i];}

    /**returns the const pointer to the ith row*/
    inline const Base* operator [] (int i) const {return _dvalues[i];}

    /**destructs the storage*/
    ~_Array2DAllocator(){
      deallocate();
    }
    
    /**constructs an rows_ by columns_ 2D array*/
    _Array2DAllocator(int rows_=0, int cols_=0) {
      allocate (rows_, cols_);
    }

    /**copy constructor*/
    _Array2DAllocator(const _Array2DAllocator<0,0,Base>& src){
      allocate (src.rows(), src.cols());
      copy(src._dvalues);
    }

    /**assignment*/
    _Array2DAllocator<0,0,Base>& operator = (const _Array2DAllocator<0,0,Base>& src){
      resize( src._rows, src._cols);
      copy(src._dvalues);
      return *this;
    }

    /**number of rows*/
    int rows() const {return _rows;}

    /**number of cols*/
    int cols() const {return _cols;}

    /**resizes the array. The content is destroyed.*/
    void resize (int n, int m) {
      if (n==rows() && m==cols())
	return;
      deallocate();
      allocate(n,m);
    }

 protected:
    Base ** _dvalues;
    void deallocate() {
      if (rows()&&cols()){
	for (int r=0; r<rows(); r++)
	  delete [] _dvalues[r];
	delete [] _dvalues;
      }
      _rows=0;
      _cols=0;
      _dvalues=0;
    }

    void allocate(int n, int m){
      if (m==0 || n==0){
	_rows=0;
	_cols=0;
	_dvalues=0;
	return;
      }
      _rows=n;
      _cols=m;
      _dvalues = new Base*[_rows];
      for (int r=0; r<_rows; r++)
	_dvalues[r]=new Base[_cols];
    }

    void copy(Base** const src){
      for (int r=0; r<rows(); r++) {
	if ( AISNAV_IS_POD(Base) )
	  memcpy(_dvalues[r], src[r], _cols*sizeof(Base));
	else
	  for  (int c=0; c<_cols; c++)
	    _dvalues[r][c]=src[r][c];
      }
    }
    int _cols;
    int _rows;
};

/**static 2 dynamic 2d array*/
template <int M, int N, typename Base>
  void st2dyn (_Array2DAllocator<0,0,Base>& dest, const _Array2DAllocator<M,N,Base>& src) {
    dest.resize(src.rows(), src.cols());
    for (int r=0; r<src.rows(); r++)
      for (int c=0; c<src.cols(); c++)
        dest[r][c]=src[r][c];
  }

/**dynamic 2 static 2d array*/
template <int M, int N, typename Base>
  void dyn2st(_Array2DAllocator<M,N,Base>& dest, const _Array2DAllocator<0,0,Base>& src){
    assert (dest.rows()==src.rows());
    assert (dest.cols()==src.cols());
    for (int r=0; r<src.rows(); r++)
      for (int c=0; c<src.cols(); c++)
        dest[r][c]=src[r][c];
  }

//@}
#endif

