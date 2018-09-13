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

#define RET_OK			  0
#define RET_NG_AREA		-10
#define RET_NG_INPUT_ARRAY_DATA	-20
#define RET_NG_OTHER		-90

//グローバル変数
//area
AreaLimits gl_varea;
//mesh size
float gl_msize;
//mesh
Delete_Noise *gl_mesh = NULL;

//mesh初期化関数
//処理に時間がかかるので、流用できるようにした
int makeMesh(std::vector<std::string> & param){
	int ret=RET_OK;
	char* argv;
	char* para;
	char* val;
	string st;
	int area_flag = 0;

	gl_msize=0.1;

	for (auto item : param){
		st = item;

		argv = new char[st.size() + 1]; // メモリ確保

		std::char_traits<char>::copy(argv, st.c_str(), st.size() + 1);
	
		if(strncmp(argv,"area=",5) == 0){ //area
			area_flag = 0;
			para=&argv[5];	

			//printf("### para=%s\n",para);

			int start = 0;
			for(int i=0;i<(int)strlen(para);i++){
				if(para[i] == '(' ){
					start = i;
				}
				if(para[i] == ')' ){
					int len = i - (start + 1); 

					val = new char[len + 1];
					strncpy(val,&para[start+1],len);

					if(area_flag == 0){ //X
						gl_varea.xmin=atof(val); val=strchr(val,',')+1;
						gl_varea.xmax=atof(val);
						area_flag = 1;
					}else if(area_flag == 1){ //Y
						gl_varea.ymin=atof(val); val=strchr(val,',')+1;
						gl_varea.ymax=atof(val);
						area_flag = 2;
					}else if(area_flag == 2){ //Z
						gl_varea.zmin=atof(val); val=strchr(val,',')+1;
						gl_varea.zmax=atof(val);
						area_flag = 3;
						break;
					}
					start=i;
				}
			}	
		}
		else if(strncmp(argv,"mesh=",5) == 0){ //mesh
			para=&argv[5];

			//printf("para=%s\n",para);

			if((int)strlen(para) > 0){
		 		gl_msize=atof(para);
			}
		}
		if(area_flag != 3){
			ret = RET_NG_AREA;
		}
		
	}

	//---DEBUG-------
	/*
	printf("gl_varea.xmin=[%f] gl_varea.xmax=[%f] gl_varea.ymin=[%f] gl_varea.ymax=[%f] gl_varea.zmin=[%f] gl_varea.zmax=[%f] gl_msize=%f\n",
		gl_varea.xmin, gl_varea.xmax, gl_varea.ymin, gl_varea.ymax, gl_varea.zmin, gl_varea.zmax,gl_msize);
	*/
	//---DEBUG-------

	//make mesh
	if(ret==RET_OK){
		if(gl_mesh == NULL){
			gl_mesh = new Delete_Noise;
			printf("allocating mesh....\n");
		}
		gl_mesh->mk_mesh(&gl_varea, gl_msize);
	}

	return ret;
}

auto loadPLY(char* FileName){
	//retcode
	int ret=RET_OK;
	//normalize後の点群配列(x,y,zの順でNx3の配列)
	py::array_t<double>scene = py::array_t<double>(1);

	try{
		ret = read_ply_from_file_to_array(FileName,&scene);
	}catch(...){
		ret = RET_NG_OTHER;
	}
	return py::make_tuple(ret, &scene);
}

//点群配列をnormalizeする
auto normalize(py::array_t<double>scene) {
	//retcode
	int ret=RET_OK;
	//normalize後の点群配列(x,y,zの順でNx3の配列)
	py::array_t<double>pc = py::array_t<double>(1);

	Point *dp=NULL;
	int dn;
	int r=2, dir=3;
	int th[2]={0,0};

	//sceneを読み込む
	dp=read_ply_from_array(scene, &dn);

	if(dp==NULL) {
		fprintf(stderr,"read array data error.\n");
		ret = RET_NG_INPUT_ARRAY_DATA;
	}

	//---DEBUG-------
	/*
	printf("gl_varea.xmin=[%f] gl_varea.xmax=[%f] gl_varea.ymin=[%f] gl_varea.ymax=[%f] gl_varea.zmin=[%f] gl_varea.zmax=[%f] gl_msize=%f\n",
			        gl_varea.xmin, gl_varea.xmax, gl_varea.ymin, gl_varea.ymax, gl_varea.zmin, gl_varea.zmax,gl_msize);
	*/
	//---DEBUG-------

	if(ret == 0){
		gl_mesh->set_points(dp,dn);
		if(__isnan(gl_varea.xmin)) {
			get_minmax(dp,dn,&gl_varea);
		}
		//mesh.mk_mesh(&varea,msize);

		fprintf(stderr,"dist_pc().\n");
		gl_mesh->dist_pc(th[0],th[1]);
		fprintf(stderr,"set_param().\n");
		gl_mesh->set_params(r,dir);
		gl_mesh->work(0);
		//if(smth!=0.0) mesh.smooth(smth);
		int rn;
		Point *rslt=gl_mesh->output(&rn);

		ret = make_ply(rslt,rn,&pc);

		delete[] rslt;
		//mesh.deleteMesh();
	}

	if(dp != NULL){
		delete[] dp;
	}

	return py::make_tuple(ret, pc);
}

//pybind
PYBIND11_MODULE(yodpy2, m) {
        m.doc() = "Normalize python library";

        m.def("makeMesh", &makeMesh, "Normalize from PLY file");
        m.def("loadPLY", &loadPLY, "Normalize from PLY file");
        m.def("normalize", &normalize, "Normalize from PLY file");
}

