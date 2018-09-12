#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "point.h"
#include "plyio.h"

Point *read_ply(char *fname,int *dn) {
	printf("read_ply fname[%s]\n",fname);

	const int minsize=sizeof(float)*3;
	FILE *fp=fopen(fname,"rb");

	//if(!fp) return NULL;
	if(!fp){
		printf("read_ply error!!\n");
		return NULL;
	}

	char lbuf[1024];
	int isasc=0;
	int psize=sizeof(float)*3;

	*dn=0;
	do {
		if(NULL==fgets(lbuf,sizeof(lbuf),fp)) return NULL;
		char *cp=strstr(lbuf,"element vertex");
		if(cp) *dn=atoi(cp+15);
		cp=strstr(lbuf,"ascii");
		if(cp) isasc=1;
		cp=strstr(lbuf,"red");
		if(cp) ++psize;
		cp=strstr(lbuf,"green");
		if(cp) ++psize;
		cp=strstr(lbuf,"blue");
		if(cp) ++psize;
		cp=strstr(lbuf,"alpha");
		if(cp) ++psize;
	} while(strncmp(lbuf,"end_header",10));
	printf("input dn=%d,psize=%d\n",*dn,psize);

	Point *dp=new Point[*dn];
	for(int i=0; i<*dn; i++) {
		if(isasc) {
			if(psize==minsize) {
				fscanf(fp,"%f%f%f",&dp[i].x,&dp[i].y,&dp[i].z);
				dp[i].r=dp[i].g=dp[i].b=255;
			}
			else if(psize==minsize+3) {
				int r,g,b;
				fscanf(fp,"%f%f%f%d%d%d",&dp[i].x,&dp[i].y,&dp[i].z,&r,&g,&b);
				dp[i].r=r;
				dp[i].g=g;
				dp[i].b=b;
				dp[i].a=255;
			}
			else {
				int r,g,b,a;
				fscanf(fp,"%f%f%f%d%d%d%d",&dp[i].x,&dp[i].y,&dp[i].z,&r,&g,&b,&a);
				dp[i].r=r;
				dp[i].g=g;
				dp[i].b=b;
				dp[i].a=a;
				printf("%f %f %f %d %d %d %d\n",dp[i].x,dp[i].y,dp[i].z,dp[i].r,dp[i].g,dp[i].b,dp[i].a);
			}
		}
		else {
			fread((void*)&dp[i],psize,1,fp);
		}
	}
	fclose(fp);
	return dp;
}

//normalize後の点群を作成する。
int make_ply(Point *dp,int dn,py::array_t<double>*pc) {
	printf("make_ply dn=%d\n",dn);
	int ret = 0;

	try{
		//pcのサイズ変更
		*pc = py::array_t<double>(dn*3);

		printf("pc size=%d\n",(int)pc->size());

		auto buf = pc->request();
		double *ptr = (double*)buf.ptr;
		int cur_pos = 0;
		for (int i=0; i < dn; i++)
		{
			ptr[cur_pos] = dp[i].x;
			cur_pos += 1;
			ptr[cur_pos] = dp[i].y;
			cur_pos += 1;
			ptr[cur_pos] = dp[i].z;
			cur_pos += 1;
		}
	}catch(...){
		ret = -1;
	}
	return ret;
}
