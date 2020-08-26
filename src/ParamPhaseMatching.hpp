#pragma once
#include <map>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>

#ifndef M_PI_2
#define M_PI_2	(0.5 * M_PI)
#endif

#ifdef YAML_PARAM
#include <yaml-cpp/yaml.h>
#endif

/**
 * 位相マッチングパラメータ
 */
struct PhaseMatchingParameter {
	double max_ph_diff;		///< 視差計算時の左右カメラの最大位相差(これを越す位相差はNG)[rad]	
	double max_parallax;	///< 最大視差[pixel]
	double min_parallax;	///< 最小視差[pixel]
	int ls_points;			///< 視差を求める際の最小二乗近似点数[points](3 or 5)

	PhaseMatchingParameter() :
		max_ph_diff(M_PI_2), max_parallax(400), min_parallax(-300), ls_points(3) {}

	PhaseMatchingParameter(const PhaseMatchingParameter &obj) {
		this->max_ph_diff = obj.max_ph_diff;
		this->max_parallax = obj.max_parallax;
		this->min_parallax = obj.min_parallax;
		this->ls_points = obj.ls_points;
	}

	PhaseMatchingParameter operator=(const PhaseMatchingParameter &obj) {
		this->max_ph_diff = obj.max_ph_diff;
		this->max_parallax = obj.max_parallax;
		this->min_parallax = obj.min_parallax;
		this->ls_points = obj.ls_points;
		return *this;
	}

	void set(std::map<std::string, double> &params) {
		if (params.count("max_ph_diff")) this->max_ph_diff = params["max_ph_diff"];
		if (params.count("max_parallax")) this->max_parallax = params["max_parallax"];
		if (params.count("min_parallax")) this->min_parallax = params["min_parallax"];
		if (params.count("ls_points")) this->ls_points = (int)params["ls_points"];
	}

#ifdef YAML_PARAM
	void set(const YAML::Node &params) {
		this->max_ph_diff = params["max_ph_diff"].as<double>();
		this->max_parallax = params["max_parallax"].as<double>();
		this->min_parallax = params["min_parallax"].as<double>();
		this->ls_points = params["ls_points"].as<int>();
	}
#endif

	/**
	 * パラメータがアルゴリズムの許容範囲に収まっているかどうかをチェックする.
	 * @return 問題なければtrue, 問題あればfalse.
	 */
	bool check(void) const {
		// 今は3 or 5しか対応していない
		if (this->ls_points == 3 || this->ls_points == 5) {}
		else return false;

		// 最大視差<最小視差である
		if (this->max_parallax < this->min_parallax) return false;
		return true;
	}
};
