#pragma once

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#include <math.h>
#endif

#ifndef M_PI_2
#define M_PI_2	(0.5 * M_PI)
#endif

/**
 * 位相シフトパラメータ
 */
struct PSFTParameter {
	int bw_diff;			///< 暗くて精度の出ない点しきい値[輝度]
	int brightness;			///< ハレーション気味で精度の出ない点しきい値[輝度]
	int darkness;			///< ハレーション気味で精度の出ない点しきい値[輝度]
	double step_diff;		///< 位相連結時のずれ修正値(phase)[rad]
	double max_step;		///< 視差画像において隣り合うピクセル間視差の最大値
	double max_ph_diff;		///< 視差計算時の左右カメラの最大位相差(これを越す位相差はNG)[rad]	
	double max_tex_diff;	///< 位相一致ピクセルの輝度差最大値[輝度]
	double max_parallax;	///< 最大視差[pixel]
	double min_parallax;	///< 最小視差[pixel]
	int right_dup_cnt;		///< 視差計算時の同一右ポイントが何回指定できるか
	int ls_points;			///< 視差を求める際の最小二乗近似点数[points](3 or 5)

	PSFTParameter() :
		bw_diff(12), brightness(256), darkness(15), step_diff(1.2),
		max_step(1.0), max_ph_diff(M_PI_2), max_tex_diff(0.7), max_parallax(400), min_parallax(-300),
		right_dup_cnt(2), ls_points(3) {}

	PSFTParameter(const PSFTParameter &obj) {
		this->bw_diff = obj.bw_diff;
		this->brightness = obj.brightness;
		this->darkness = obj.darkness;
		this->step_diff = obj.step_diff;
		this->max_step = obj.max_step;
		this->max_ph_diff = obj.max_ph_diff;
		this->max_tex_diff = obj.max_tex_diff;
		this->max_parallax = obj.max_parallax;
		this->min_parallax = obj.min_parallax;
		this->right_dup_cnt = obj.right_dup_cnt;
		this->ls_points = obj.ls_points;
		if (this->ls_points == 3 || this->ls_points == 5) return;
		this->ls_points = 3;
	}

	PSFTParameter operator=(const PSFTParameter &obj) {
		this->bw_diff = obj.bw_diff;
		this->brightness = obj.brightness;
		this->darkness = obj.darkness;
		this->step_diff = obj.step_diff;
		this->max_step = obj.max_step;
		this->max_ph_diff = obj.max_ph_diff;
		this->max_tex_diff = obj.max_tex_diff;
		this->max_parallax = obj.max_parallax;
		this->min_parallax = obj.min_parallax;
		this->right_dup_cnt = obj.right_dup_cnt;
		this->ls_points = obj.ls_points;
		if (this->ls_points == 3 || this->ls_points == 5) return *this;
		this->ls_points = 3;
		return *this;
	}
};
