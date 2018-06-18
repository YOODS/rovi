//
//  common.h
//  PhaseShift
//
//  Created by 原田 寛 on 2018/02/25.
//  Copyright © 2018年 原田 寛. All rights reserved.
//

#ifndef common_h
#define common_h

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

extern char *getline(char *line,int len,FILE *fp);
extern void save_pgm(char *fn,Eigen::Matrix<uchar,Eigen::Dynamic,Eigen::Dynamic> &data);
extern void save_pgm(char *fn,Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> &data,double offset =0,double gain =1.0);
extern void read_pgm(char *fn,Eigen::Matrix<uchar,Eigen::Dynamic,Eigen::Dynamic> &data);
extern void save_ppm(char *fn,Eigen::Matrix<unsigned long,Eigen::Dynamic,Eigen::Dynamic> &data);


#define TIME_POINT              std::chrono::system_clock::time_point
#define SET_TPOINT              std::chrono::system_clock::now()
#define GET_DTIME(start,end)    std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count()

#endif /* common_h */
