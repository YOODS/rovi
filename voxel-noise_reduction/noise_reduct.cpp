#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "noise_reduct.h"
#include "plyio.h"
#include "mycommons.h"

Delete_Noise::Delete_Noise(void) {
	dirf=1;
	r=1;
}

void Delete_Noise::set_params(int rm, int fn) {
	dirf=fn;
	r=rm;
}

void Delete_Noise::work(void*) {
	dist_pc();
	Voxel::work(NULL);
	int deln=0;

	vn=0;
	for(int i=r; i<cx-r; i++) {
		for(int j=r; j<cy-r; j++) {
			for(int k=0; k<cz; k++) {
				if(msh[i][j][k].count()==0) continue;
				int f=dirf;
				for(int p=i-r; p<=i+r; p++) {
					if(msh[p][j-r][k].count()) {
						--f;
						break;
					}
				}
				for(int p=i-r; p<=i+r && f; p++) {
					if(msh[p][j+r][k].count()) {
						--f;
						break;
					}
				}
				for(int p=j-r; p<=j+r && f; p++) {
					if(msh[i-r][p][k].count()) {
						--f;
						break;
					}
				}
				for(int p=j-r; p<=j+r && f; p++) {
					if(msh[i+r][p][k].count()) {
						--f;
						break;
					}
				}
				if(f) {
					msh[i][j][k].clean();
					deln++;
				}
				else {
					vp[vn++]=*(msh[i][j][k].c->p);
				}
			}
		}
	}
	printf("deleted points=%d, remained points=%d\n", deln,vn);
}


