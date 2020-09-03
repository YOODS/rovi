#pragma once

#include "ParamPhaseMatching.hpp"
#include <map>

#ifdef YAML_PARAM
#include <yaml-cpp/yaml.h>
#endif


/**
 * 多位相シフトパラメータ
 */
struct MPhaseDecodeParameter : public PhaseMatchingParameter {
	int datatype;		///< 入力が画像の場合 = 0, 入力が白黒画像＆位相データの場合 = 1, その他はエラー

	int n_phaseshift;	///< 位相のシフト数
	int n_periods;		///< 周期の数(種類数)
	int period[4];		///< period[i] : i番目の周期の値

	int brightness;		///< 最大輝度値(この値以上の画素は処理対象としない)
	int darkness;		///< 最小輝度値(この値以下の画素は処理対象としない)
	int bw_diff;		///< 輝度差閾値(この値未満の画素は処理対象としない)
	
	MPhaseDecodeParameter()	: PhaseMatchingParameter(), datatype(0),
		n_phaseshift(4), n_periods(3),
		brightness(256), darkness(15), bw_diff(16)
	{
		period[0] = 7;
		period[1] = 11;
		period[2] = 17;
		period[3] = 0;
	}

	MPhaseDecodeParameter(const MPhaseDecodeParameter &obj) : PhaseMatchingParameter(obj)
	{
		this->datatype = obj.datatype;

		this->brightness = obj.brightness;
		this->darkness = obj.darkness;
		this->bw_diff = obj.bw_diff;

		this->n_phaseshift = obj.n_phaseshift;
		this->n_periods = obj.n_periods;
		this->period[0] = obj.period[0];
		this->period[1] = obj.period[1];
		this->period[2] = obj.period[2];
		this->period[3] = obj.period[3];
	}

	MPhaseDecodeParameter operator=(const MPhaseDecodeParameter &obj)
	{
		PhaseMatchingParameter::operator=(obj);

		this->datatype = obj.datatype;

		this->brightness = obj.brightness;
		this->darkness = obj.darkness;
		this->bw_diff = obj.bw_diff;

		this->n_phaseshift = obj.n_phaseshift;
		this->n_periods = obj.n_periods;
		this->period[0] = obj.period[0];
		this->period[1] = obj.period[1];
		this->period[2] = obj.period[2];
		this->period[3] = obj.period[3];

		return *this;
	}

	void set(std::map<std::string, double> &params)
	{
		reinterpret_cast<PhaseMatchingParameter*>(this)->set(params);

		if (params.count("datatype")) this->datatype = (int)params["datatype"];

		if (params.count("brightness")) this->brightness = (int)params["brightness"];
		if (params.count("darkness")) this->darkness = (int)params["darkness"];
		if (params.count("bw_diff")) this->bw_diff = (int)params["bw_diff"];

		if (params.count("n_phaseshift")) this->n_phaseshift = (int)params["n_phaseshift"];
		if (params.count("n_periods")) this->n_periods = (int)params["n_periods"];
		if (params.count("period0")) this->period[0] = (int)params["period0"];
		if (params.count("period1")) this->period[1] = (int)params["period1"];
		if (params.count("period2")) this->period[2] = (int)params["period2"];
		if (params.count("period3")) this->period[3] = (int)params["period3"];
	}

#ifdef YAML_PARAM
	void set(const YAML::Node &params)
	{
		reinterpret_cast<PhaseMatchingParameter*>(this)->set(params);
		if (params["datatype"]) this->datatype = params["datatype"].as<int>();
		
		if (params["brightness"]) this->brightness = params["brightness"].as<int>();
		if (params["darkness"]) this->darkness = params["darkness"].as<int>();
		if (params["bw_diff"]) this->bw_diff = params["bw_diff"].as<int>();

		if (params["n_phaseshift"]) this->n_phaseshift = params["n_phaseshift"].as<int>();

		if (params["periods"]) {
			this->n_periods = params["periods"].size();
			for (int n = 0; n < this->n_periods; n++) {
				this->period[n] = params["periods"][n].as<int>();
			}
		}
		else this->n_periods = 0;
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
		
		// 今の所、シフト数は3 or 4しか対応していない
		if (n_phaseshift == 3 || n_phaseshift == 4) {}
		else return false;
		
#ifdef ORIGINAL
		// 周期の組み合わせは4まで(ヘッダの都合上)
		if (n_periods < 2 || n_periods >= 4) return false;
#else
		// 周期の組み合わせは3でなければならない
		if (n_periods != 3) return false;
#endif
		return true;
	}
};
