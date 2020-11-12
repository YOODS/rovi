#pragma once
#include "ParamPhaseMatching.hpp"
#include <map>

#if 0
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#include <math.h>
#endif

#ifndef M_PI_2
#define M_PI_2	(0.5 * M_PI)
#endif
#endif

#ifdef YAML_PARAM
#include <yaml-cpp/yaml.h>
#endif

/**
 * グレイコード＋位相シフトパラメータ
 */
struct GPhaseDecodeParameter : public PhaseMatchingParameter {
	int datatype;			///< 入力が画像の場合 = 0, 入力が白黒画像＆位相・グレイコードデータの場合 = 1, その他はエラー

	int bw_diff;			///< 明暗差[輝度差].全点灯画像-全消灯画像の輝度差が、この値未満であれば計算から除外される.
	int brightness;			///< 全点灯画像においてハレーション気味で精度の出ない点の閾値[輝度].この値よりも大きな画素は除外される.
	int darkness;			///< 全点灯画像において暗くて精度の出ない点しきい値[輝度].この値よりも小さな画素は除外される.

	int phase_wd_min;		///< 画像内の一周期分の最小幅(画素数). この値よりも小さい幅の区間は確実なデータとしない.無効にしたい場合は0を指定すること.
	int phase_wd_thr;		///< ゴミとして捨てる区間の最大幅. この値以下の区間は捨てられる.無効にしたい場合は0にすること.
	int gcode_variation;	///< 一区間に含まれるグレイコード種類数の最大値. 1以上3以下

	GPhaseDecodeParameter() 
		: PhaseMatchingParameter(), datatype(0),
		bw_diff(16), brightness(256), darkness(15),
		phase_wd_min(8), phase_wd_thr(3), gcode_variation(3)
	{
	}

	GPhaseDecodeParameter(const GPhaseDecodeParameter &obj) {
		this->datatype = obj.datatype;

		this->bw_diff = obj.bw_diff;
		this->brightness = obj.brightness;
		this->darkness = obj.darkness;

		this->phase_wd_min = obj.phase_wd_min;
		this->phase_wd_thr = obj.phase_wd_thr;
		this->gcode_variation = obj.gcode_variation;
	}

	GPhaseDecodeParameter operator=(const GPhaseDecodeParameter &obj) {
		PhaseMatchingParameter::operator=(obj);

		this->datatype = obj.datatype;

		this->bw_diff = obj.bw_diff;
		this->brightness = obj.brightness;
		this->darkness = obj.darkness;

		this->phase_wd_min = obj.phase_wd_min;
		this->phase_wd_thr = obj.phase_wd_thr;
		this->gcode_variation = obj.gcode_variation;
		return *this;
	}

	void set(std::map<std::string, double> &params) {
		reinterpret_cast<PhaseMatchingParameter*>(this)->set(params);

		if (params.count("datatype")) this->datatype = (int)params["datatype"];

		if (params.count("bw_diff")) this->bw_diff = (int)params["bw_diff"];
		if (params.count("brightness")) this->brightness = (int)params["brightness"];
		if (params.count("darkness")) this->darkness = (int)params["darkness"];

		if (params.count("phase_wd_min")) this->phase_wd_min = (int)params["phase_wd_min"];
		if (params.count("phase_wd_thr")) this->phase_wd_thr = (int)params["phase_wd_thr"];
		if (params.count("gcode_variation")) this->gcode_variation = (int)params["gcode_variation"];
	}

#ifdef YAML_PARAM
	void set(const YAML::Node &params) {
		reinterpret_cast<PhaseMatchingParameter*>(this)->set(params);

		if (params["datatype"]) this->datatype = params["datatype"].as<int>();

		if (params["bw_diff"]) this->bw_diff = params["bw_diff"].as<int>();
		if (params["brightness"]) this->brightness = params["brightness"].as<int>();
		if (params["darkness"]) this->darkness = params["darkness"].as<int>();

		if (params["phase_wd_min"]) this->phase_wd_min = params["phase_wd_min"].as<int>();
		if (params["phase_wd_thr"]) this->phase_wd_thr = params["phase_wd_thr"].as<int>();
		if (params["gcode_variation"]) this->gcode_variation = params["gcode_variation"].as<int>();
	}
#endif

	/**
	 * パラメータがアルゴリズムの許容範囲に収まっているかどうかをチェックする.
	 * @return 問題なければtrue, 問題あればfalse.
	 */
	bool check(void) const {
		if (!PhaseMatchingParameter::check()) return false;

		// 画像か位相データか指定されていなければ駄目
		if (datatype != 0 && datatype != 1) return false;

		// 負の値は受け付けない
		if (phase_wd_min < 0) return false;
		if (phase_wd_thr < 0) return false;
		if (gcode_variation < 1) return false;

		return true;
	}
};



