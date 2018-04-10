//
//  PointCloud.h
//  PhaseShift
//
//  Created by 原田 寛 on 2018/03/15.
//  Copyright © 2018年 原田 寛. All rights reserved.
//

#ifndef PointCloud_h
#define PointCloud_h

#include <Eigen/Dense>

struct PointCloud{
    float coord[3];
    uchar col[4];
};

namespace Eigen {
    typedef Matrix<uchar,Eigen::Dynamic,Eigen::Dynamic> MatrixXp;
    typedef Matrix<PointCloud,Eigen::Dynamic,Eigen::Dynamic> MatrixXpt;
}

#endif /* PointCloud_h */
