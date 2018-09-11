#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "noise_reduct.h"
#include "plyio.h"
#include "mycommons.h"


#include <pybind11/pybind11.h>
#include <pybind11/numpy.h> // numpy
#include <pybind11/stl.h> // vector用
#include <pybind11/eigen.h>


namespace py = pybind11;
using namespace std;

int normalize(std::vector<std::string> & param) {
	char *outfn=NULL,*infn=NULL;
	int ascf=0;
	int notexf=0;
	unsigned char color[3]={0,0,0};
	float msize=0.1;
	Delete_Noise mesh;
	AreaLimits varea;
	Point *dp;
	int dn;
	int r=2, dir=3;
	int th[2]={0,0};
	double smth=0.0;

	//printf("param size=%d\n",(int)param.size());

	for (auto item : param){
		string st = item;

		char* argv = new char[st.size() + 1]; // メモリ確保

		std::char_traits<char>::copy(argv, st.c_str(), st.size() + 1);

		//printf("argv=%s\n",argv);

		if(argv[0]=='-') {
			if(argv[1]=='-') {
				if(!strncmp(&argv[2],"notexture",5)) {
					//printf("option notexture\n");
					notexf=1;
				}
			}
			else if(argv[1]=='a') {
				ascf=1;
				//printf("ascf = 1\n");
			}
			else if(argv[1]=='c') {
				int j;
				for(j=2; j < (int)strlen(argv)-2 ; j++){
					if(argv[j] != ' '){
						break;
					}
				}
				char *p=&argv[j];

				if((int)strlen(p) > 0){
					for(int k=0; k<3; k++){
						color[k]=atox(&p[k*2],2);
					}
					//printf("color[0]=[%x] color[1]=[%x] color[2]=[%x]\n",color[0],color[1],color[2]);
				}

//				char *arg=&argv[2];
//				for(int j=0; j<3; j++)
//					color[j]=atox(&arg[j*2],2);
			}
			else if(argv[1]=='o') {
				int j;
				for(j=2; j < (int)strlen(argv)-2 ; j++){
					if(argv[j] != ' '){
						break;
					}
				}
				char *p=&argv[j];

				if((int)strlen(p) > 0){
					outfn = new char[strlen(p) + 1];
					strcpy(outfn,p);
					//printf("outfn=[%s]\n",outfn);
				}
			}
			else if(argv[1]=='m') { // mesh size(mm)
				int j;
				for(j=2; j < (int)strlen(argv)-2 ; j++){
					if(argv[j] != ' '){
						break;
					}
				}
				char *p=&argv[j];

				if((int)strlen(p) > 0){
					msize=atof(&argv[j]);
					//printf("msize=[%f]\n",msize);
				}
			}
			else if(argv[1]=='r') { // noise judge(pixel)
				int j;
				for(j=2; j < (int)strlen(argv)-2 ; j++){
					if(argv[j] != ' '){
						break;
					}
				}
				char *p=&argv[j];
				if((int)strlen(p) > 0){
					r=atoi(&argv[j]);
					//printf("r=[%d]\n",r);
				}
			}
			else if(argv[1]=='d') { // noise judge(directions)
				int j;
				for(j=2; j < (int)strlen(argv)-2 ; j++){
					if(argv[j] != ' '){
						break;
					}
				}
				char *p=&argv[j];

				if((int)strlen(p) > 0){
					dir=atoi(&argv[j]);
					//printf("dir=[%d]\n",dir);
				}
			}
			else if(argv[1]=='t') { // texture limits
				int j;
				for(j=2; j < (int)strlen(argv)-2 ; j++){
					if(argv[j] != ' '){
						break;
					}
				}
				char *p=&argv[j];

				if((int)strlen(p) > 0){
					th[0]=atoi(p); p=strchr(p,',')+1;
					th[1]=atoi(p);
					//printf("th[0]=[%d] th[1]=[%d]\n", th[0],th[1]);
				}

//				if(argv[2]) sscanf(&argv[2],"%d,%d",&th[0],&th[1]);
//				else sscanf(argv[3],"%d,%d",&th[0],&th[1]);
			}
			else if(argv[1]=='s') { // smooth
				int j;
				for(j=2; j < (int)strlen(argv)-2 ; j++){
					if(argv[j] != ' '){
						break;
					}
				}
				char *p=&argv[j];

				if((int)strlen(p) > 0){
					smth=atof(&argv[j]);
					//printf("smth=[%f]\n",smth);
				}
			}
			else if(argv[1]=='A') { // area
				int j;
				for(j=2; j < (int)strlen(argv)-2 ; j++){
					if(argv[j] != ' '){
						break;
					}
				}
				char *p=&argv[j];

				if((int)strlen(p) > 0){
					varea.xmin=atof(p); p=strchr(p,',')+1;
					varea.xmax=atof(p); p=strchr(p,',')+1;
					varea.ymin=atof(p); p=strchr(p,',')+1;
					varea.ymax=atof(p); p=strchr(p,',')+1;
					varea.zmin=atof(p); p=strchr(p,',')+1;
					varea.zmax=atof(p); p=strchr(p,',')+1;
					//printf("varea.xmin=[%f] varea.xmax=[%f] varea.ymin=[%f] varea.ymax=[%f] varea.zmin=[%f] varea.zmax=[%f]\n",
					//        varea.xmin, varea.xmax, varea.ymin, varea.ymax, varea.zmin, varea.zmax);
				}
			}
		}
		else {
			infn = new char[strlen(argv) + 1];
			strcpy(infn,argv);
			if(!outfn) outfn=infn;
			//printf("infn=[%s]\n",infn);
		}

		delete[] argv; // メモリ解放
	}

	//printf("call read_ply infn=[%s]\n",infn);
	//printf("call read_ply outfn=[%s]\n",outfn);

	if(infn==NULL) {
		fprintf(stderr,"No input file.\n");
		return -1;
	}
	dp=read_ply(infn, &dn);
	if(dp==NULL) {
		fprintf(stderr,"read file error(%s).\n", infn);
		return -1;
	}
	mesh.set_points(dp,dn);
	if(__isnan(varea.xmin)) {
		get_minmax(dp,dn,&varea);
	}
	mesh.mk_mesh(&varea,msize);

//for(int i=0; i<1; i++) {
	fprintf(stderr,"dist_pc().\n");
	mesh.dist_pc(th[0],th[1]);
	fprintf(stderr,"set_param().\n");
	mesh.set_params(r,dir);
	mesh.work(0);
	if(smth!=0.0) mesh.smooth(smth);
	int rn;
	Point *rslt=mesh.output(&rn);
	write_ply(rslt,rn,outfn,ascf,notexf,(color[0]|color[1]|color[2]) ? color:NULL);
	delete[] rslt;
	mesh.deleteMesh();
//}
	delete[] dp;

	delete[] infn;
	delete[] outfn;

	return 0;
}

//pybind
PYBIND11_MODULE(yodpy2, m) {
        m.doc() = "Normalize python library";

        m.def("normalize", &normalize, "Normalize from PLY file");
}

