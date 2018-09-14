#include <stdio.h>
#include "point_mesh.h"

PointChain::PointChain(Point *a) {
	p=a;
	next=NULL;
}

PointChain* PointChain::add(Point *ap) {
	if(p==NULL) {
		p=ap;
		return this;
	}
	if(next) return next->add(ap);
	else {
		next=new PointChain(ap);
		next->p=ap;
	}
	return next;
}

int PointChain::count(void) {
	if(next) return next->count()+1;
	else return p ? 1: 0;
}

PointChain::~PointChain(void) {
	if(next) delete next;
}

PCMesh::PCMesh(void) {
	c=new PointChain(NULL);
}

PCMesh::~PCMesh(void) {
	clean();
}

PointChain* PCMesh::add(Point* ap) {
	return c->add(ap);
};

int PCMesh::count(void) {
	return c->count();
}

void PCMesh::clean(void) {
	delete c;
	c=new PointChain(NULL);
}
