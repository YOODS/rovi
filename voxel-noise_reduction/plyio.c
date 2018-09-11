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

void write_ply(Point *dp,int dn,char *outfn,int ascf,int notexf,unsigned char *color) {
	printf("outputfile=%s, n=%d\n",outfn,dn);
	FILE *fp=fopen(outfn,"wb");
	fprintf(fp,"ply\n");
	if(ascf) {
		fprintf(fp,"format ascii 1.0\n");

	}
	else {
		fprintf(fp,"format binary_little_endian 1.0\n");
	}
	fprintf(fp,"comment VCGLIB generated\n");
	fprintf(fp,"element vertex %d\n",dn);
	fprintf(fp,"property float x\n");
	fprintf(fp,"property float y\n");
	fprintf(fp,"property float z\n");
	if(!notexf) {
		fprintf(fp,"property uchar red\n");
		fprintf(fp,"property uchar green\n");
		fprintf(fp,"property uchar blue\n");
	}
	fprintf(fp,"element face 0\n");
	fprintf(fp,"property list uchar int vertex_indices\n");
	fprintf(fp,"end_header\n");
	for(int i=0; i<dn; i++) {
		if(color) {
			dp[i].r=color[0];
			dp[i].g=color[1];
			dp[i].b=color[2];
		}
		if(ascf) {
			if(notexf) {
				fprintf(fp,"%f %f %f\n",dp[i].x,dp[i].y,dp[i].z);
			}
			else {
				fprintf(fp,"%f %f %f %d %d %d\n",dp[i].x,dp[i].y,dp[i].z,dp[i].r,dp[i].g,dp[i].b);
			}
		}
		else {
			if(notexf) {
				fwrite(&dp[i],sizeof(float)*3,1,fp);
			}
			else {
				fwrite(&dp[i],sizeof(float)*3+3,1,fp);
			}
		}
	}
	fclose(fp);
}
