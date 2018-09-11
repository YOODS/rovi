#include <stdio.h>
#include "point_mesh.h"
#include "mycommons.h"
#include "voxel.h"

Voxel::Voxel(void) {
	pp=NULL;
	pn=0;
	unitl=0;
}

Voxel::~Voxel(void) {}

void Voxel::work(void*) {
	int n=0;
	for(int i=0; i<cx; i++) {
		for(int j=0; j<cy; j++) {
			for(int k=0; k<cz; k++) {
				int cnt=count(i,j,k);
				if(cnt!=0) {
					double ax=0,ay=0,az=0,ar=0,ag=0,ab=0;
					PointChain *c=msh[i][j][k]->c;
					for(int a=0; a<cnt; a++) {
						ax+=c->p->x;
						ay+=c->p->y;
						az+=c->p->z;
						ar+=c->p->r;
						ag+=c->p->g;
						ab+=c->p->b;
						c=c->next;
					}
					msh[i][j][k]->c->p->x=ax/cnt;
					msh[i][j][k]->c->p->y=ay/cnt;
					msh[i][j][k]->c->p->z=az/cnt;
					msh[i][j][k]->c->p->r=ar/cnt;
					msh[i][j][k]->c->p->g=ag/cnt;
					msh[i][j][k]->c->p->b=ab/cnt;
					delete msh[i][j][k]->c->next;
					msh[i][j][k]->c->next=NULL;
					++n;
				}
			}
		}
	}
}

void Voxel::smooth(double r) {
	double r2=pow(r,2);
	for(int i=0; i<pn; i++) {
		int x=(pp[i].x-area.xmin)/unitl;
		int y=(pp[i].y-area.ymin)/unitl;
		int z=(pp[i].z-area.zmin)/unitl;
		if (x < 0 || y < 0 || z < 0) continue;
		if (x<cx && y<cy && z<cz && count(x,y,z)) {
			PointChain *pc=get_neighbor(x,y,z);
			double ax=0,ay=0,az=0,ar=0,ag=0,ab=0;
			int n=0;
			for(PointChain *c=pc; c; c=c->next) {
				if(get_distance2(&pp[i],c->p)<r2) {
					ax+=c->p->x;
					ay+=c->p->y;
					az+=c->p->z;
					ar+=c->p->r;
					ag+=c->p->g;
					ab+=c->p->b;
					++n;
				}
			}
			if(n>0) {
				pp[i].x=ax/n;
				pp[i].y=ay/n;
				pp[i].z=az/n;
				pp[i].r=ar/n;
				pp[i].g=ag/n;
				pp[i].b=ab/n;
			}
 			delete pc;
		}
	}
}

void Voxel::mk_mesh(AreaLimits *a,float l) {
	if(msh) deleteMesh();
	area=*a;
	unitl=l;
	if(__isnan(area.zmax)) return;
	cx=(area.xmax-area.xmin)/unitl;
	cy=(area.ymax-area.ymin)/unitl;
	cz=(area.zmax-area.zmin)/unitl;
	fprintf(stderr,"mk_mesh:meshsize=%d,%d,%d msize=%.2g\n",cx,cy,cz,unitl);
	if(allocMesh(cx,cy,cz)) {
		fprintf(stderr,"can't allocate mesh memory\n");
		return;
	}
	fprintf(stderr,"mk_mesh finished.\n");
}

void Voxel::dist_pc(int min,int max) {
	if(!msh) {
		fprintf(stderr,"msh is NULL. dist_pc should be called after mk_mesh().\n");
		return;
	}
	printf("pn=%d\n", pn);
	for (int i = 0; i<pn; i++) {
		int x=(pp[i].x-area.xmin)/unitl;
		int y=(pp[i].y-area.ymin)/unitl;
		int z=(pp[i].z-area.zmin)/unitl;
		if (x < 0 || y < 0 || z < 0) continue;
		if(min!=max && (pp[i].r<min || pp[i].r>max)) continue;
		if (x<cx && y<cy && z<cz) {
			if(!msh[x][y][z]) {
				msh[x][y][z]=new PCMesh;
			}
			msh[x][y][z]->add(&pp[i]);
		}
	}
	mn=0;
	for(int i=0; i<cx; i++) {
		for(int j=0; j<cy; j++) {
			for(int k=0; k<cz; k++) {
				if(msh[i][j][k]) ++mn;
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

/*
 * 使った後はPointをdeleteすること
 */
Point *Voxel::output(int *rn) {
	*rn=0;
	for(int i=0; i<cx; i++) {
		for(int j=0; j<cy; j++) {
			for(int k=0; k<cz; k++) {
				*rn+=count(i,j,k);
			}
		}
	}
	Point *rs=new Point[*rn];
	*rn=0;
	for(int i=0; i<cx; i++) {
		for(int j=0; j<cy; j++) {
			for(int k=0; k<cz; k++) {
				int n=count(i,j,k);
				if(n==0) continue;
				PointChain *pc=msh[i][j][k]->c;
				for(int l=0; l<n; l++) {
					rs[(*rn)++]=*(pc->p);
					pc=pc->next;
				}
			}
		}
	}
	return rs;
}

/*
 * 使った後はPointChainをdeleteすること
 */
PointChain *Voxel::get_neighbor(int x, int y, int z,int pix) {
	PointChain *ret=NULL, *endp=NULL;
	for(int i=x+pix; i>=x-pix; i--) {
		if(i<0) break;
		for(int j=y+pix; j>=y-pix; j--) {
			if(j<0) break;
			for(int k=z+pix; k>=z-pix; k--) {
				if(k<0) break;
				if(count(i,j,k)==0) continue;
				endp=msh[i][j][k]->c->copy(endp);
				if(ret==NULL) ret=endp;
			}
		}
	}
// 	int i=0;
// 	for(PointChain *a=ret; a; a=a->next)
// 		printf("%d(%g,%g,%g)\n",i++,a->p->x,a->p->y,a->p->z);
	return ret;
}


int Voxel::work1(void) {return 0;}
