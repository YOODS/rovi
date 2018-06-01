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

#define PH_SEARCH_DIV   5			//視差階層探索
#define BW_DIFF			15			//暗くて精度の出ない点しきい値
#define BRIGHTNESS	  	254			//ハレーション気味で精度の出ない点しきい値
#define MAX_PH_DIFF		M_PI_2		//視差計算時の左右カメラの最大位相差(これを越す位相差はNG)(rad)
#define MAX_PARALLAX	400			//最大視差(rad)
#define MIN_PARALLAX	-200		//最小視差
#define MAX_TEX_DIFF	100			//視差から得られた左右同一点の最大輝度差[0-255]
#define LS_POINTS		3			//視差を求める際の最小二乗近似点数
#define SPECKLE_RANGE	5			//位相画像からスペックルノイズを除去する際の平均化範囲
#define SPECKLE_PHASE	M_PI_4		//位相画像中のスペックルノイズしきい値(phase)
#define SPECKLE_PIXEL	20			//視差画像中のスペックルノイズしきい値(pixel)

struct PS_PARAMS {
	int search_div;
	int bw_diff;
	int brightness;
	double max_ph_diff;
	double max_parallax;
	double min_parallax;
	int max_tex_diff;
	int speckle_range;
	double speckle_phase;
	double speckle_pixel;
	int ls_points;
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
	Eigen::MatrixXd _bg;
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
	void chk_code7(void);
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

