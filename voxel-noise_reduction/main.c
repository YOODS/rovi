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


auto normalize(std::vector<std::string> & param) {
	//retcode
	int ret=0;
	//normalize後の点群配列(x,y,zの順でNx3の配列)
	py::array_t<double>pc = py::array_t<double>(1);

	char *infn=NULL;
	float msize=0.1;
	Delete_Noise mesh;
	AreaLimits varea;
	Point *dp=NULL;
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
			if(argv[1]=='m') { // mesh size(mm)
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
			//printf("infn=[%s]\n",infn);
		}

		delete[] argv; // メモリ解放
	}

	if(infn==NULL) {
		fprintf(stderr,"No input file.\n");
		ret = -1;
	}
	
	if(ret == 0){
		dp=read_ply(infn, &dn);
		if(dp==NULL) {
			fprintf(stderr,"read file error(%s).\n", infn);
			ret = -1;
		}
	}

	if(ret == 0){
		mesh.set_points(dp,dn);
		if(__isnan(varea.xmin)) {
			get_minmax(dp,dn,&varea);
		}
		mesh.mk_mesh(&varea,msize);

		fprintf(stderr,"dist_pc().\n");
		mesh.dist_pc(th[0],th[1]);
		fprintf(stderr,"set_param().\n");
		mesh.set_params(r,dir);
		mesh.work(0);
		if(smth!=0.0) mesh.smooth(smth);
		int rn;
		Point *rslt=mesh.output(&rn);

		ret = make_ply(rslt,rn,&pc);

		delete[] rslt;
		mesh.deleteMesh();
	}

	if(dp != NULL){
		delete[] dp;
	}

	delete[] infn;

	return py::make_tuple(ret, pc);
}

//pybind
PYBIND11_MODULE(yodpy2, m) {
        m.doc() = "Normalize python library";

        m.def("normalize", &normalize, "Normalize from PLY file");
}

