#pragma once

#include <stdio.h>
#include <stdlib.h>

template <typename T>
class Mesh3D {
protected:
	T ***msh;
	int cx,cy,cz;
public:
	Mesh3D(void) {
		cx=cy=cz=0;
		msh=NULL;
	}
	Mesh3D(int p,int q,int r) {
		msh=NULL;
		allocMesh(p,q,r);
	}
	~Mesh3D(void) { deleteMesh(); }
	int allocMesh(int p,int q,int r) {
		if(msh) return -1;
		msh=(T***)calloc(p,sizeof(T**));
		for(int i=0; i<p; i++) {
			msh[i]=(T**)calloc(q,sizeof(T*));
			for(int j=0; j<q; j++) {
				msh[i][j]=new T[r];
			}
		}
		cx=p; cy=q; cz=r;
		return 0;
	}
	void deleteMesh(void) {
		if(!msh) return;
		for(int i=0; i<cx; i++){
			for(int j=0; j<cy; j++){
				delete[] msh[i][j];
			}
			free(msh[i]);
		}
		free(msh);
		msh=NULL;
		cx=cy=cz=0;
		fprintf(stderr,"Mesh3D::deleteMesh() finished!\n");
	}
	void clearMesh(void) {
		for(int i=0; i<cx; i++){
			for(int j=0; j<cy; j++){
				for(int k=0; k<cz; k++){
					msh[i][j][k].clean();
				}
			}
		}
		fprintf(stderr,"Mesh3D::clearMesh() finished!\n");
	}
	T& operator()(int i,int j,int k) {
		return msh[i][j][k];
	}
	virtual void work(void*) {};
};