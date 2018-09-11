#include <stdio.h>
#include "noise_reduct.h"

Delete_Noise::Delete_Noise(void) {
	dirf=1;
	r=1;
}

void Delete_Noise::set_params(int rm, int fn) {
	dirf=fn;
	r=rm;
}

void Delete_Noise::work(void*) {
	work1();
}

int Delete_Noise::work1(void) {
	int deln=0;
	int rn=0;
	for(int i=r; i<cx-r; i++) {
		for(int j=r; j<cy-r; j++) {
			for(int k=0; k<cz; k++) {
				if(count(i,j,k)==0) continue;
				int f=dirf;
				for(int p=i-r; p<=i+r; p++) {
					if(count(p,j-r,k)) {
						--f;
						break;
					}
				}
				for(int p=i-r; p<=i+r && f; p++) {
					if(count(p,j+r,k)) {
						--f;
						break;
					}
				}
				for(int p=j-r; p<=j+r && f; p++) {
					if(count(i-r,p,k)) {
						--f;
						break;
					}
				}
				for(int p=j-r; p<=j+r && f; p++) {
					if(count(i+r,p,k)) {
						--f;
						break;
					}
				}
				if(f) {
					msh[i][j][k]->clean();
					deln++;
				}
				else {
					rn++;
				}
			}
		}
	}
	printf("deleted voxel=%d, remained voxel=%d\n", deln,rn);
	return rn;
}
