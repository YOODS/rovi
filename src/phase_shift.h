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
	NG_CODE,
	NG_PHASE,
	NG_NOMATCH,
	NG_MAX_PH_DIFF,
	NG_MAX_TEX_DIFF,
	NG_MAX_CD_DIFF,
	NG_PARALLAX,
	NG_MAX_STEP,
	NG_NAN,
	NG_GENPC,
};

#define PH_SEARCH_DIV   4			//視差階層探索
#define BW_DIFF			12			//暗くて精度の出ない点しきい値[輝度]
#define BRIGHTNESS		256			//ハレーション気味で精度の出ない点しきい値[輝度]
#define DARKNESS		15			//ハレーション気味で精度の出ない点しきい値[輝度]
#define STEP_DIFF		1.2			//位相連結時のずれ修正値(phase)[rad]
#define MAX_PH_DIFF		M_PI_2		//視差計算時の左右カメラの最大位相差(これを越す位相差はNG)[rad[
#define MAX_TEX_DIFF	0.7			//位相一致ピクセルの輝度差最大値[輝度]
#define MAX_PARALLAX	400			//最大視差[pixel]
#define MIN_PARALLAX	-300		//最小視差[pixel]
#define RIGHT_DUP_N		2			//視差計算時の同一右ポイントが何回指定できるか
#define LS_POINTS		3			//視差を求める際の最小二乗近似点数[points]
#define EVEC_ERROR		2.0e-13		//視線ベクトルを用いて点群生成する際の視線誤差閾値
#define MAX_STEP		1.0			//視差画像において隣り合うピクセル間視差の最大値
#define CHECKBLOCK_SIZE	8			//コードノイズ除去判定用ウインドウサイズ

struct PS_PARAMS {
	int search_div;
	int bw_diff;
	int brightness;
	int darkness;
	double step_diff;
	double max_step;
	double max_ph_diff;
	double max_tex_diff;
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
	void subtract_bg(Eigen::MatrixXp &tg,Eigen::MatrixXp &bg);
	void subtract_bg(Eigen::MatrixXd &tg,Eigen::MatrixXd &bg);
protected:
	void check_code7(int cam);
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

