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
		if(i<15){
			printf("read_ply dp[%d].x=%f dp[%d].y=%f dp[%d].z=%f dp[%d].a=%d\n"
				,i,dp[i].x,dp[i].y,dp[i].z,dp[i].a);
		}

	}
	fclose(fp);
	return dp;
}

Point *read_ply_from_array(py::array_t<double>scene,int *dn) {

	try{
		//***********************************
		//sceneからPointを作る
		//***********************************
		//scenelはx,y,zの順で並んでいるので、各要素の集合に分ける。(この並びで良いのかは不明)
		ssize_t point_num = scene.size() / 3;

		*dn = point_num;

		Point *dp=new Point[point_num];

		printf("read_ply_from_array scene size=%d\n",(int)scene.size());
		printf("read_ply_from_array scene point num=%d\n",(int)point_num);
		printf("read_ply_from_array dn=%d\n",*dn);
		ssize_t current_pos = 0;

		for(ssize_t i=0; i < scene.size(); i++){
			dp[current_pos].r=dp[current_pos].g=dp[current_pos].b=255;
			dp[current_pos].x = *scene.data(i);

			//---DEBUG------
			/*
			if(i<15){
				printf("read_ply_from_array dp[%d].x=%f\n",current_pos,*scene.data(i));
			}
			*/
			//---DEBUG------
			
			i += 1;
			dp[current_pos].y = *scene.data(i);

			//---DEBUG------
			/*
			if(i<15){
				printf("read_ply_from_array dp[%d].y=%f\n",current_pos,*scene.data(i));
			}
			*/
			//---DEBUG------

			i += 1;
			dp[current_pos].z = *scene.data(i);
			current_pos++;

			//---DEBUG------
			/*
			if(i<15){
				printf("read_ply_from_array dp[%d].z=%f\n",current_pos,*scene.data(i));
			}
			*/
			//---DEBUG------
		}

		return dp;
	}catch(...){
		return NULL;
	}
}

//テスト用
int read_ply_from_file_to_array(char *fname,py::array_t<double>*scene) {
	printf("read_ply_from_file_to_array fname[%s]\n",fname);
	int ret = 0;

	const int minsize=sizeof(float)*3;
	FILE *fp=fopen(fname,"rb");

	if(!fp){
		printf("read_ply error!!\n");
		ret = -1;
	}

	if(ret == 0){

		char lbuf[1024];
		int isasc=0;
		int psize=sizeof(float)*3;

	
		try{
			int dn=0;
			do {
				if(NULL==fgets(lbuf,sizeof(lbuf),fp)) return NULL;
				char *cp=strstr(lbuf,"element vertex");
				if(cp) dn=atoi(cp+15);
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
			printf("input dn=%d,psize=%d\n",dn,psize);

			Point *dp=new Point[dn];

			//sceneのサイズ変更
			*scene = py::array_t<double>(dn*3);

			printf("scene size=%d\n",(int)scene->size());

			auto buf = scene->request();
			double *ptr = (double*)buf.ptr;
			int cur_pos = 0;

			for(int i=0; i<dn; i++) {
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

				//---DEBUG--------
				/*
				if(i<15){
					printf("read_ply_from_file_to_array dp[%d].x=%f dp[%d].y=%f dp[%d].z=%f dp[%d].a=%d\n"
						,i,dp[i].x,dp[i].y,dp[i].z,dp[i].a);
				}
				*/
				//---DEBUG--------

				//ここでsceneにx,y,zデータを設定
				ptr[cur_pos] = dp[i].x;
				cur_pos += 1;
				ptr[cur_pos] = dp[i].y;
				cur_pos += 1;
				ptr[cur_pos] = dp[i].z;
				cur_pos += 1;

			}
			fclose(fp);
		}catch(...){
			ret = -1;
		}

	}

	return ret;
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
