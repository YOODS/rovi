#pragma once

#include <math.h>

inline float __nan(void){ long x=-1; return *(float *)&x;}
inline int __isnan(float x){ long *l=(long *)&x; long m=0x7c000000; return (*l&m)==m;}

struct Point {
	float x,y,z;
	unsigned char r,g,b,a;
};

struct AreaLimits {
	float xmin,xmax,ymin,ymax,zmin,zmax;
	AreaLimits(void) {
		xmin=__nan(); xmax=__nan();
		ymin=__nan(); ymax=__nan();
		zmin=__nan(); zmax=__nan();
	}
};
