#include "grid_line_traversal.h"

void GridLineTraversal::gridLineCore(Eigen::Vector2i start, Eigen::Vector2i end, GridLineTraversalLine *line ) {
  int dx, dy, incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;
  int cnt = 0;

  dx = std::abs(end.x() - start.x());
  dy = std::abs(end.y() - start.y());
  
  if(dy <= dx) {
    d = 2 * dy - dx; 
    incr1 = 2 * dy; 
    incr2 = 2 * (dy - dx);
    if(start.x() > end.x()) {
      x = end.x(); 
      y = end.y();
      ydirflag = (-1);
      xend = start.x();
    } 
    else {
      x = start.x(); 
      y = start.y();
      ydirflag = 1;
      xend = end.x();
    }
    line->points[cnt].x() = x;
    line->points[cnt].y() = y;
    cnt++;
    if(((end.y() - start.y()) * ydirflag) > 0) {
      while(x < xend) {
	x++;
	if(d < 0) {
	  d += incr1;
	} 
	else {
	  y++; 
	  d += incr2;
	}
	line->points[cnt].x() = x;
	line->points[cnt].y() = y;
	cnt++;
      }
    } 
    else {
      while(x < xend) {
	x++;
	if(d < 0) {
	  d += incr1;
	} 
	else {
	  y--; 
	  d += incr2;
	}
	line->points[cnt].x() = x;
	line->points[cnt].y() = y;
	cnt++;
      }
    }		
  } 
  else {
    d = 2 * dx - dy;
    incr1 = 2 * dx; 
    incr2 = 2 * (dx - dy);
    if(start.y() > end.y()) {
      y = end.y(); 
      x = end.x();
      yend = start.y();
      xdirflag = (-1);
    } 
    else {
      y = start.y(); 
      x = start.x();
      yend = end.y();
      xdirflag = 1;
    }
    line->points[cnt].x() = x;
    line->points[cnt].y() = y;
    cnt++;
    if(((end.x() - start.x()) * xdirflag) > 0) {
      while(y < yend) {
	y++;
	if(d < 0) {
	  d += incr1;
	} 
	else {
	  x++; 
	  d += incr2;
	}
	line->points[cnt].x()=x;
	line->points[cnt].y()=y;
	cnt++;
      }
    } 
    else {
      while(y < yend) {
	y++;
	if(d < 0) {
	  d += incr1;
	} 
	else {
	  x--; 
	  d += incr2;
	}
	line->points[cnt].x() = x;
	line->points[cnt].y() = y;
	cnt++;
      }
    }
  }
  line->numPoints = cnt;
}

void GridLineTraversal::gridLine(Eigen::Vector2i start, Eigen::Vector2i end, GridLineTraversalLine *line) {
  int i, j;
  int half;
  Eigen::Vector2i v;
  gridLineCore(start, end, line);
  if(start.x() != line->points[0].x() || start.y() != line->points[0].y()) {
    half = line->numPoints / 2;
    for(i = 0, j = line->numPoints - 1; i < half; i++, j--) {
      v = line->points[i];
      line->points[i] = line->points[j];
      line->points[j] = v;
    }
  }
}
