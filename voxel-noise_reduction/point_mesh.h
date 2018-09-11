#pragma once

#include "point.h"

struct PointChain {
	Point *p;
	PointChain *next;
	PointChain(Point *a);
	PointChain *add(Point *ap);
	PointChain *copy(PointChain *ac);
	int count(void);
	~PointChain(void);
};

struct PCMesh {
	PointChain *c;
	PCMesh(void);
	~PCMesh(void);
	PointChain* add(Point* ap);
	int count(void);
	void clean(void);
};
