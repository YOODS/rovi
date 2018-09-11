#pragma once

#include "mesh3d.h"

class Voxel: public Mesh3D<PCMesh> {
protected:
	Point *pp;		// original points
	int pn;			// original points count
	AreaLimits area;// area for voxel
	float unitl;	// unit length of 1 voxel
	int mn;			// mesh count include points
public:
	Voxel(void);
	~Voxel(void);
	void work(void*);
	void smooth(double r);
	virtual int work1(void);
	void mk_mesh(AreaLimits *a,float l);
	void dist_pc(int min =0,int max =0);
	void set_points(Point *a, int n);
	Point *get_points(int n =0);
	int get_count(void);
	PointChain *get_neighbor(int x, int y, int z,int pix =1);
	Point *output(int *rn);
};
