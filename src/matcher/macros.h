#ifndef MACROS_H
#define MACROS_H

#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * 0.01745329251994329575)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 57.29577951308232087721)
#endif

// some macros that are only useful for c++
#ifdef __cplusplus

#define READ_FILE(fvar,fname) \
   std::ifstream fvar(fname);\
   if (!fvar) throw std::runtime_error("Could not open file for reading: "#fname);
#define WRITE_FILE(fvar,fname) \
   std::ofstream fvar(fname);\
   if (!fvar) throw std::runtime_error("Could not open file for writing "#fname);
#define WRITE_VRML_FILE(fvar,fname) \
   std::ofstream fvar(fname);\
   if (!fvar) throw std::runtime_error("Could not open file for writing "#fname);\
   fvar << "#VRML V2.0 utf8\n";
#define FSKIP_LINE(f) \
   {char c=' ';while(c != '\n' && f.good() && !(f).eof()) (f).get(c);}
#define FSKIP_WHITESPACES(f) \
   {char c; do f.get(c); while(isspace(c) && f.good() && !(f).eof()); f.unget();}
#define FCOPY_LINE(f, g) \
   {char _c=' ';while(_c != '\n' && !(f).eof()){ (f).get(_c); (g).put(_c);}}
#define FGET_LINE(f,l) \
   {l = ""; char c=' '; while(c!='\n' && !f.eof()) l+=c=f.get();}
#define DEL_FEXT(fname) \
   {std::string::size_type d = fname.find_last_of('.'); \
    if (d!=std::string::npos) fname = fname.substr(0,d);}
#define GET_FEXT(fname,fext) \
   {std::string::size_type d = fname.find_last_of('.'); \
    fext = (d!=std::string::npos) ? fname.substr(d+1) : "";}
#define TO_UPPER(s) \
  std::transform(s.begin(), s.end(), s.begin(), (int(*)(int)) std::toupper)
#define TO_LOWER(s) \
  std::transform(s.begin(), s.end(), s.begin(), (int(*)(int)) std::tolower)

#ifndef PVAR
  #define PVAR(s) \
    #s << " = " << (s) << std::flush
#endif
#ifndef PVARN
#define PVARN(s) \
  #s << " = " << (s) << "\n"
#endif
#ifndef PVARC
#define PVARC(s) \
  #s << " = " << (s) << ", " << std::flush
#endif
#ifndef PVARS
#define PVARS(s) \
  #s << " = " << (s) << " " << std::flush
#endif
#ifndef PVARA
#define PVARA(s) \
  #s << " = " << RAD2DEG(s) << "deg" << std::flush
#endif
#ifndef PVARAN
#define PVARAN(s) \
  #s << " = " << RAD2DEG(s) << "deg\n"
#endif
#ifndef PVARAC
#define PVARAC(s) \
  #s << " = " << RAD2DEG(s) << "deg, " << std::flush
#endif
#ifndef PVARAS
#define PVARAS(s) \
  #s << " = " << RAD2DEG(s) << "deg " << std::flush
#endif

#define FIXED(s) \
  std::fixed << s << std::resetiosflags(std::ios_base::fixed)

#endif // __cplusplus

#ifndef GET_COLOR
#define GET_COLOR(i, size, r, g, b) \
  ((floor(i*6./(double)size) == 0) || (floor(i*6./(double)size) == 6) ? \
   (r=1,g=(i*6./size-floor(i*6./size)),b=0) : \
   ((floor(i*6./(double)size) == 1) ? (r=1.-(i*6./size-floor(i*6./size)),g=1,b=0) : \
    ((floor(i*6./(double)size) == 2) ? (r=0,g=1,b=(i*6./size-floor(i*6./size))) : \
     ((floor(i*6./(double)size) == 3) ? (r=0,g=1-(i*6./size-floor(i*6./size)), b=1) : \
      ((floor(i*6./(double)size) == 4) ? (r=(i*6./size-floor(i*6./size)),g=0, b=1) : \
       (r=1,g=0, b=1-(i*6./size-floor(i*6./size))))))))
#endif

#define PREC 1e-12
#define SQRT2 1.414213562373095145474621858738828450441
#define SQRT3 1.732050807568877293527446341505872366943

#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif

#ifndef MIN3
#define MIN3(a,b,c) MIN((a),MIN((b),(c)))
#endif

#ifndef MIN4
#define MIN4(a,b,c,d) MIN((d),MIN3((a),(b),(c)))
#endif

#ifndef MAX
#define MAX(a,b) ((a)>(b)?(a):(b))
#endif

#ifndef MAX3
#define MAX3(a,b,c) MAX((a),MAX((b),(c)))
#endif

#ifndef MAX4
#define MAX4(a,b,c,d) MAX((d),MAX3((a),(b),(c)))
#endif

#ifndef SQR
#define SQR(x) ((x)*(x))
#endif

#ifndef CUB
#define CUB(x) ((x)*(x)*(x))
#endif

#ifndef HYPOT
#define HYPOT(x,y) (sqrt(SQR(x) + SQR(y)))
#endif

#ifndef HYPOT_SQUARED
#define HYPOT_SQUARED(x,y) (SQR(x) + SQR(y))
#endif

#ifndef ROUND
#define ROUND(x) ((x) < 0 ? (int)((x) - .5): (int) ((x) + .5))
#endif

#ifndef IS_INT
#define IS_INT(a) (fabs(ROUND(a)-(a))<PREC)
#endif

#ifndef SIGN
#define SIGN(a) ((a)>0?1:((a)<0?-1:0))
#endif

#ifndef IS_ZERO
#define IS_ZERO(f) (fabs(f) < 1e-12)
#endif

#ifndef IS_EQUAL
#define IS_EQUAL(f, g) (fabs(f - g) < 1e-12)
#endif

#ifndef ERASE_STRUCT
#define ERASE_STRUCT(var) memset(&var, 0, sizeof(var))
#endif

#ifndef ERASE_ARRAY
#define ERASE_ARRAY(var, size) memset(var, 0, size*sizeof(*var))
#endif

#ifndef SET_ARRAY
#define SET_ARRAY(var, value, size) {for (int i=0; i<(int)size; ++i) var[i]=value;}
#endif

// Helper macros for builtin compiler support.
// If your compiler has builtin support for any of the following
// traits concepts, then redefine the appropriate macros to pick
// up on the compiler support:
//
// (these should largely ignore cv-qualifiers)
// AIS_IS_POD(T) should evaluate to true if T is a POD type

// gcc >= 4.3
#if defined(__GNUC__) && ((__GNUC__ > 4) || ((__GNUC__ == 4) && (__GNUC_MINOR__ >= 3) && !defined(__GCCXML__)))
#define AISNAV_IS_POD(T) __is_pod(T)
#endif

// default compiler
#ifndef AISNAV_IS_POD
#define AISNAV_IS_POD(T) (0)
#endif

#endif
