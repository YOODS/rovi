#pragma once
#include <map>


/**
 * 位相シフトパラメータ
 */
struct PSFTParameter {
	int bw_diff;			///< 明暗差[輝度差].全点灯画像と全消灯画像でこれ以上輝度差が無い画素は計算から除外される.
	int brightness;			///< 全点灯画像においてハレーション気味で精度の出ない点の閾値[輝度]
	int darkness;			///< 全点灯画像において暗くて精度の出ない点しきい値[輝度]

	int phase_wd_min;		///< 画像内の一周期分の最小幅(画素数). この値よりも小さい幅の区間は確実なデータとしない
	int phase_wd_thr;		///< ゴミとして捨てる区間の最大幅. この値以下の区間は捨てられる
	int gcode_variation;	///< 一区間に含まれるグレイコード種類数の最大値. 1以上3以下

	double max_ph_diff;		///< 視差計算時の左右カメラの最大位相差(これを越す位相差はNG)[rad]	
	double max_parallax;	///< 最大視差[pixel]
	double min_parallax;	///< 最小視差[pixel]
	int ls_points;			///< 視差を求める際の最小二乗近似点数[points](3 or 5)
	int right_dup_cnt;		///< 視差計算時の同一右ポイントが何回指定できるか

	PSFTParameter() :
		bw_diff(12), brightness(256), darkness(15), 
		phase_wd_min(5), phase_wd_thr(2), gcode_variation(3), 
		max_ph_diff(M_PI_2), max_parallax(400), min_parallax(-300), ls_points(3), right_dup_cnt(3) {}

	PSFTParameter(const PSFTParameter &obj) {
		this->bw_diff = obj.bw_diff;
		this->brightness = obj.brightness;
		this->darkness = obj.darkness;

		this->phase_wd_min = obj.phase_wd_min;
		this->phase_wd_thr = obj.phase_wd_thr;
		this->gcode_variation = obj.gcode_variation;

		this->max_ph_diff = obj.max_ph_diff;
		this->max_parallax = obj.max_parallax;
		this->min_parallax = obj.min_parallax;
		this->ls_points = obj.ls_points;
		this->right_dup_cnt = obj.right_dup_cnt;


		if (this->ls_points == 3 || this->ls_points == 5) {}
		else {
			// 3 or 5でなければ強制的に3にする
			this->ls_points = 3;
		}
	}

	PSFTParameter operator=(const PSFTParameter &obj) {
		this->bw_diff = obj.bw_diff;
		this->brightness = obj.brightness;
		this->darkness = obj.darkness;

		this->phase_wd_min = obj.phase_wd_min;
		this->phase_wd_thr = obj.phase_wd_thr;
		this->gcode_variation = obj.gcode_variation;

		this->max_ph_diff = obj.max_ph_diff;
		this->max_parallax = obj.max_parallax;
		this->min_parallax = obj.min_parallax;
		this->ls_points = obj.ls_points;
		this->right_dup_cnt = obj.right_dup_cnt;
		if (this->ls_points == 3 || this->ls_points == 5) {}
		else {
			// 3 or 5でなければ強制的に3にする
			this->ls_points = 3;
		}
		return *this;
	}

	void set(std::map<std::string, double> &params) {
		if (params.count("bw_diff")) this->bw_diff = (int)params["bw_diff"];
		if (params.count("brightness")) this->brightness = (int)params["brightness"];
		if (params.count("darkness")) this->darkness = (int)params["darkness"];

		if (params.count("phase_wd_min")) this->phase_wd_min = (int)params["phase_wd_min"];
		if (params.count("phase_wd_thr")) this->phase_wd_thr = (int)params["phase_wd_thr"];
		if (params.count("gcode_variation")) this->gcode_variation = (int)params["gcode_variation"];

		if (params.count("max_ph_diff")) this->max_ph_diff = params["max_ph_diff"];
		if (params.count("max_parallax")) this->max_parallax = params["max_parallax"];
		if (params.count("min_parallax")) this->min_parallax = params["min_parallax"];
		if (params.count("ls_points")) this->ls_points = (int)params["ls_points"];
		if (params.count("right_dup_cnt")) this->right_dup_cnt = (int)params["right_dup_cnt"];

		if (this->ls_points == 3 || this->ls_points == 5) {}
		else {
			// 3 or 5でなければ強制的に3にする
			this->ls_points = 3;
		}
	}
};
