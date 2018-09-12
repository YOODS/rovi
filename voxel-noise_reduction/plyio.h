#pragma once 

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h> // numpy
#include <pybind11/stl.h> // vector用
#include <pybind11/eigen.h>

namespace py = pybind11;
using namespace std;

extern Point *read_ply(char *fname,int *dn);

//テスト用--------------------------------
extern int read_ply_from_file_to_array(char *fname,py::array_t<double>*scene);
//----------------------------------------

extern Point *read_ply_from_array(py::array_t<double>scene,int *dn);
extern int make_ply(Point *dp,int dn,py::array_t<double>*pc);
