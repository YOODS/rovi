#pragma once

#include "mesh3d.h"

class Voxel: public Mesh3D<PCMesh> {
protected:
	Point *pp;		// original points
	int pn;			// original points count
	Point *vp;		// voxeled points
	int vn;			// voxeled points count
	AreaLimits area;// area for voxel
	float unitl;	// unit length of 1 voxel
	int mn;			// mesh count include points
public:
	Voxel(void);
	~Voxel(void);
	void work(void*);
	void mk_mesh(AreaLimits *a,float l);
	void dist_pc(void);
	void set_points(Point *a, int n);
	Point *get_points(int n =0);
	int get_count(void);
	Point *get_vpoints(void);
	int get_vcount(void);
};
