#pragma once

#include "point_mesh.h"
#include "voxel.h"

class Delete_Noise: public Voxel {
	int dirf;	// 4 angles
	int r;		// distance from center [pixel]
public:
	Delete_Noise(void);
	void set_params(int rm, int fn);
	void work(void*);
	int work1(void);
};

