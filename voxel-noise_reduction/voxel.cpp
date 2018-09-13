#include <stdio.h>
#include "point_mesh.h"
#include "voxel.h"

Voxel::Voxel(void) {
	pp=vp=NULL;
	pn=vn=0;
	unitl=0;
}

Voxel::~Voxel(void) { if(vp) delete[] vp; vp=NULL; }

void Voxel::work(void*) {
	if(vp) delete[] vp;
	vp=new Point[vn=mn];
	int n=0;
	for(int i=0; i<cx; i++) {
		for(int j=0; j<cy; j++) {
			for(int k=0; k<cz; k++) {
				int cnt=msh[i][j][k].count();
				if(cnt!=0) {
					double ax=0,ay=0,az=0,ar=0,ag=0,ab=0;
					PointChain *c=msh[i][j][k].c;
					for(int a=0; a<cnt; a++) {
						ax+=c->p->x;
						ay+=c->p->y;
						az+=c->p->z;
						ar+=c->p->r;
						ag+=c->p->g;
						ab+=c->p->b;
						c=c->next;
					}
					vp[n].x=ax/cnt;
					vp[n].y=ay/cnt;
					vp[n].z=az/cnt;
					vp[n].r=ar/cnt;
					vp[n].g=ag/cnt;
					vp[n].b=ab/cnt;
					*(msh[i][j][k].c->p)=vp[n]; // first point for the cell should be replaced.
					++n;
				}
			}
		}
	}
}

void Voxel::mk_mesh(AreaLimits *a,float l) {
	if(msh) clearMesh();
	area=*a;
	unitl=l;
	if(__isnan(area.zmax)) return;
	cx=(area.xmax-area.xmin)/unitl;
	cy=(area.ymax-area.ymin)/unitl;
	cz=(area.zmax-area.zmin)/unitl;
	fprintf(stderr,"meshsize=%d,%d,%d\n",cx,cy,cz);
	if(allocMesh(cx,cy,cz)) {
		fprintf(stderr,"can't allocate mesh memory\n");
		return;
	}
}

void Voxel::dist_pc(void) {
	if(!msh) {
		fprintf(stderr,"msh is NULL. dist_pc should be called after mk_mesh().\n");
		return;
	}
	printf("pn=%d\n", pn);
	for (int i = 0; i<pn; i++) {
		int x=(pp[i].x-area.xmin)/unitl;
		int y=(pp[i].y-area.ymin)/unitl;
		int z=(pp[i].z-area.zmin)/unitl;
//		printf("%d: %d,%d,%d\n", i, x, y, z);
		if (x < 0 || y < 0 || z < 0) continue;
		if (x<cx && y<cy && z<cz) {
			msh[x][y][z].add(&pp[i]);
		}
//		else
//			printf("%d out %d,%d,%d\n", i, x, y, z);
	}
	mn=0;
	for(int i=0; i<cx; i++) {
		for (int j = 0; j<cy; j++) {
			for(int k=0; k<cz; k++) {
				int cnt=msh[i][j][k].count();
				if(cnt!=0) ++mn;
			}
		}
	}
}

void Voxel::set_points(Point *a, int n) {
//	if(pn) delete[] pp;
	pp=a;
	pn=n;
}

Point *Voxel::get_points(int n) {
	if(n) {
		if(pn) delete[] pp;
		pp=new Point[pn=n];
	}
	return pp;
}

int Voxel::get_count(void) { return pn; }

Point *Voxel::get_vpoints(void) { return vp; }

int Voxel::get_vcount(void) { return vn; }
