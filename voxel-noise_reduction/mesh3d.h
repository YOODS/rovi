#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

template <typename T>
class Mesh3D {
protected:
	T ****msh;
	int cx,cy,cz;
	int cnt;
public:
	Mesh3D(void) {
		cx=cy=cz=cnt=0;
		msh=NULL;
	}
	Mesh3D(int p,int q,int r) {
		msh=NULL;
		allocMesh(p,q,r);
	}
	~Mesh3D(void) { deleteMesh(); }
	int allocMesh(int p,int q,int r) {
		if(msh) return -1;
		msh=new T***[p];
		msh[0]=new T**[p*q];
		msh[0][0]=new T*[cnt=p*q*r];
		for(int i=0; i<p; i++) {
			msh[i]=msh[0]+i*q;
			for(int j=0; j<q; j++) {
				msh[i][j]=msh[0][0]+i*q*r+j*r;
			}
		}
		memset(msh[0][0],0,cnt*sizeof(T*));
		cx=p; cy=q; cz=r;
		return 0;
	}
	void deleteMesh(void) {
		if(!msh) return;
		for(int i=0; i<cx; i++) {
			for(int j=0; j<cy; j++) {
				for(int k=0; k<cz; k++) {
					if(msh[i][j][k]) msh[i][j][k]->clean();
				}
			}
		}

 		delete[] msh[0][0];
 		delete[] msh[0];
 		delete[] msh;
		msh=NULL;
		cx=cy=cz=cnt=0;
		fprintf(stderr,"Mesh3D::deleteMesh() finished!\n");
	}
	int count(int i,int j,int k) {
		if(!msh[i][j][k]) return 0;
		else return msh[i][j][k]->count();
	}
	virtual void work(void*) {};
};