//
//  phase_shift.hpp
//  PhaseShift
//
//  Created by 原田 寛 on 2018/02/25.
//  Copyright © 2018年 原田 寛. All rights reserved.
//
#ifndef phase_shift_hpp
#define phase_shift_hpp

#define DEBUG   1

#ifdef DEBUG
	#include <opencv2/opencv.hpp>
//	#include <opencv2/core/eigen.hpp>
	#include "common.h"
#endif
#include "PointCloud.h"

#define CAMN	2
#define LCAM	0
#define RCAM	1

enum {
	CAM1,CAM2
};

enum {
	Black,White,C0,C1,C2,C3,C4,C5,C6,P0,P1,P2,P3
};

enum {
	NGPT=200,
	NG_BRIGHTNESS,
	NG_MARGIN,
	NG_PHASE,
	NG_SPECKLE,
	NG_MAX_PH_DIFF,
	NG_PARALLAX,
	NG_MAX_STEP,
	NG_NAN,
	NG_GENPC,
};

#define PH_SEARCH_DIV   4
#define BW_DIFF			12
#define BRIGHTNESS		256
#define DARKNESS		15
#define STEP_DIFF		1.2
#define MAX_PH_DIFF		M_PI_2
#define MAX_PARALLAX	400
#define MIN_PARALLAX	-300
#define RIGHT_DUP_N		2
#define LS_POINTS		3
#define EVEC_ERROR		2.0e-13
#define MAX_STEP		1.0	

struct PS_PARAMS {
	int search_div;
	int bw_diff;
	int brightness;
	int darkness;
	double step_diff;
	double max_step;
	double max_ph_diff;
	double max_parallax;
	double min_parallax;
	int right_dup_cnt;
	int ls_points;
	double evec_error;
};

class StereoCamera;

class PHASE_SHIFT {
public:
	Eigen::MatrixXp mask[CAMN];
	Eigen::MatrixXpt pt;
	Eigen::MatrixXd diff;
	Eigen::MatrixXp texture;
	int ptN;
private:
	PS_PARAMS param;
	Eigen::MatrixXp _bw[CAMN][2];
	Eigen::MatrixXp _threshold;
	Eigen::MatrixXp _bin[CAMN][7];
	Eigen::MatrixXd _ph[CAMN][4];
	Eigen::MatrixXp code[CAMN];
	Eigen::MatrixXd phase[CAMN];
	Eigen::Matrix4d Q;
	int *_LLimits, *_RLimits;
	void subtract_bg(Eigen::MatrixXp &tg,Eigen::MatrixXp &bg);
	void subtract_bg(Eigen::MatrixXd &tg,Eigen::MatrixXd &bg);
protected:
	void check_brightness(int cam);
	void mk_code7(int cam);
	void mk_phase(int cam);
	void unwrap(int cam);
	void mk_diff(void);
	void genPC(void);
	void genPC(StereoCamera &cam);
public:
	PHASE_SHIFT(void);
	PHASE_SHIFT(int w,int h);
	PHASE_SHIFT(int w,int h,PS_PARAMS &p);
	void setparams(PS_PARAMS &p);
	void setsize(int w,int h);
	void setdata(int cam, int idx,uchar *d);
	void setQ(Eigen::Matrix4d &q);
	Eigen::MatrixXd& execute(void);
	int width,height,size;
#ifdef DEBUG
	void check(void*);
	Eigen::MatrixXp getmtrix(int n);
	void save_line(int cam,int r,char *fname);
#endif
};

#endif /* phase_shift_hpp */

