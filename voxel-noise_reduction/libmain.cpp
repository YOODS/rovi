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
//voxel
int gl_voxel;
//mesh
Delete_Noise *gl_mesh = NULL;

//mesh初期化関数
//処理に時間がかかるので、流用できるようにした
int makeMesh(py::kwargs kwargs){
	int ret=RET_OK;
	int key_len;
	int para_len;
	int val_len;
	char* key;
	char* para;
	char* val;
	int area_flag = 0;

	gl_msize=0.1;

	if(gl_mesh != NULL){
		delete gl_mesh;
		gl_mesh = NULL;
	}

	int i=0;
	for (auto item : kwargs){
		//key
		key_len = strlen((py::cast<string>(item.first)).c_str());
		key = new char[key_len + 1]; // メモリ確保
		std::char_traits<char>::copy(key, (py::cast<string>(py::str(item.first))).c_str(), key_len + 1);

		//param
		para_len = strlen((py::cast<string>(py::str(item.second))).c_str());
		para = new char[para_len + 1]; // メモリ確保
		std::char_traits<char>::copy(para, (py::cast<string>(py::str(item.second))).c_str(), para_len + 1);

		if(strcmp(key,"mesh")==0){ //mesh size
			if(para_len > 0){
				gl_msize = atof(para);
			}
		}
		else if(strcmp(key,"area") == 0){ //area
			int start = 0;
			for(int i=0;i<para_len;i++){
				if(para[i] == '(' ){
					start = i;
				}
				if(para[i] == ')' ){
					val_len = i - (start + 1); 

					val = new char[val_len + 1];
					strncpy(val,&para[start+1],val_len);

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
		i++;
	}

	//areaが指定されていない場合はNG
	if(area_flag != 3){
		ret = RET_NG_AREA;
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
auto normalize(py::array_t<double>scene,py::kwargs kwargs) {
	//retcode
	int ret=RET_OK;
	//normalize後の点群配列(x,y,zの順でNx3の配列)
	py::array_t<double>pc = py::array_t<double>(1);

	Point *dp=NULL;
	int dn;
	int dir=3;
	//int th[2]={0,0};

	int key_len;
	int para_len;
	char* key;
	char* para;
	gl_voxel = 0;

	//sceneを読み込む
	dp=read_ply_from_array(scene, &dn);

	if(dp==NULL) {
		fprintf(stderr,"read array data error.\n");
		ret = RET_NG_INPUT_ARRAY_DATA;
	}

	//voxel取得
	int i=0;
	for (auto item : kwargs){
		//key
		key_len = strlen((py::cast<string>(item.first)).c_str());
		key = new char[key_len + 1]; // メモリ確保
		std::char_traits<char>::copy(key, (py::cast<string>(py::str(item.first))).c_str(), key_len + 1);

		//param
		para_len = strlen((py::cast<string>(py::str(item.second))).c_str());
		para = new char[para_len + 1]; // メモリ確保
		std::char_traits<char>::copy(para, (py::cast<string>(py::str(item.second))).c_str(), para_len + 1);

		if(strcmp(key,"voxel")==0){ //voxel
			if(para_len > 0){
				gl_voxel = atoi(para);
			}
		}
		i++;
	}

	//---DEBUG-------
	/*
	printf("gl_voxel=[%d]\n",gl_voxel);
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

		fprintf(stderr,"set_param().\n");

		//gl_mesh->set_params(r,dir);
		gl_mesh->set_params(gl_voxel,dir);

		gl_mesh->work(0);
		int rn=gl_mesh->get_vcount();
		Point *rslt=gl_mesh->get_vpoints();

		ret = make_ply(rslt,rn,&pc);

		gl_mesh->clearMesh();

		//delete[] rslt;
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

        m.def("makeMesh", &makeMesh, "Make mesh");
        m.def("loadPLY", &loadPLY, "Load PLY file");
        m.def("normalize", &normalize, "Normalize from array(PLY)");
}
