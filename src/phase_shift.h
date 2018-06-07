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

#define NGPT	255

enum {
	CAM1,CAM2
};

enum {
	Black,White,C0,C1,C2,C3,C4,C5,C6,P0,P1,P2,P3
};

#define PH_SEARCH_DIV   5			//視差階層探索
#define BW_DIFF			15			//暗くて精度の出ない点しきい値[輝度]
#define BRIGHTNESS		254			//ハレーション気味で精度の出ない点しきい値[輝度]
#define DARKNESS		30			//ハレーション気味で精度の出ない点しきい値[輝度]
#define STEP_DIFF		2.4			//位相連結時のずれ修正値(phase)[rad]
#define MAX_PH_DIFF		M_PI_2		//視差計算時の左右カメラの最大位相差(これを越す位相差はNG)[rad[
#define MAX_PARALLAX	500			//最大視差[pixel]
#define MIN_PARALLAX	-500		//最小視差[pixel]
#define RIGHT_DUP_N		1			//視差計算時の同一右ポイントが何回指定できるか
#define LS_POINTS		3			//視差を求める際の最小二乗近似点数[points]
#define SPECKLE_RANGE	5			//位相画像からスペックルノイズを除去する際の平均化範囲[points]
#define SPECKLE_PHASE	M_PI_4		//位相画像中のスペックルノイズしきい値[phase]
#define SPECKLE_PIXEL	20			//視差画像中のスペックルノイズしきい値[pixel]

struct PS_PARAMS {
	int search_div;
	int bw_diff;
	int brightness;
	int darkness;
	double step_diff;
	double max_ph_diff;
	double max_parallax;
	double min_parallax;
	int rdup_cnt;
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

