#pragma once

#include <map>

#ifdef YAML_PARAM
#include <yaml-cpp/yaml.h>
#endif

/**
 * SGBMパラメータ
 */
struct SGBMParameter {
	int min_disparity;		///< 取り得る最小の視差値
	int num_disparities;	///< 視差の総数(必ず16の倍数)
	int blockSize;			///< マッチングされるブロックのサイズ(必ず奇数)
	int P1;	///< 視差の滑らかさを制御するパラメータ1(マイナスを設定しておくと妥当な値を内部で計算します)
	int P2;	///< 視差の滑らかさを制御するパラメータ2(P2 > P1)(マイナスを設定しておくと妥当な値を内部で計算します)
	int disp12MaxDiff;		///< 視差チェックにおける許容される最大の差.チェックを行わない場合は0以下の値とする.
	int preFilterCap;		///< 事前フィルタにおいて画像ピクセルを切り捨てる閾値(x微分値の範囲)
	int uniquenessRatio;	///< 最適解は二番目の解に対して、ここで指定した値よりもコスト関数値が良くなければならない(パーセント単位)
	int speckleWindowSize;	///< ノイズスペックルや無効なピクセルが考慮された滑らかな視差領域の最大サイズ.0にするとスペックルフィルタが無効になる
	int speckleRange;		///< それぞれの連結成分における最大視差(1or2が適切な値)
	int mode;	///< SGBMのモード(0: MODE_SGBM, 1: MODE_HH, 2: MODE_SGBM_3WAY, 3: MODE_HH4. 1にすると完全な2パス動的計画法にて探索を行う.が、画像サイズx視差数分のメモリが必要なので注意)

	SGBMParameter() :
		min_disparity(0),
		num_disparities(256),
		blockSize(5),
		P1(-1),
		P2(-1),
		disp12MaxDiff(0),
		preFilterCap(0),
		uniquenessRatio(20),
		speckleWindowSize(100),
		speckleRange(1),
		mode(0)	{}

	SGBMParameter(const SGBMParameter &obj) {
		this->min_disparity = obj.min_disparity;
		this->num_disparities = obj.num_disparities;
		this->blockSize = obj.blockSize;
		this->P1 = obj.P1;
		this->P2 = obj.P2;
		this->disp12MaxDiff = obj.disp12MaxDiff;
		this->preFilterCap = obj.preFilterCap;
		this->uniquenessRatio = obj.uniquenessRatio;
		this->speckleWindowSize = obj.speckleWindowSize;
		this->speckleRange = obj.speckleRange;
		this->mode = obj.mode;
	}

	SGBMParameter operator=(const SGBMParameter &obj) {
		this->min_disparity = obj.min_disparity;
		this->num_disparities = obj.num_disparities;
		this->blockSize = obj.blockSize;
		this->P1 = obj.P1;
		this->P2 = obj.P2;
		this->disp12MaxDiff = obj.disp12MaxDiff;
		this->preFilterCap = obj.preFilterCap;
		this->uniquenessRatio = obj.uniquenessRatio;
		this->speckleWindowSize = obj.speckleWindowSize;
		this->speckleRange = obj.speckleRange;
		this->mode = obj.mode;
		return (*this);
	}

	void set(std::map<std::string, double> &params) {
		if (params.count("min_disparity")) this->min_disparity = (int)params["min_disparity"];
		if (params.count("num_disparities")) this->num_disparities = (int)params["num_disparities"];
		if (params.count("blockSize")) this->blockSize = (int)params["blockSize"];
		if (params.count("P1")) this->P1 = (int)params["P1"];
		if (params.count("P2")) this->P2 = (int)params["P2"];
		if (params.count("disp12MaxDiff")) this->disp12MaxDiff = (int)params["disp12MaxDiff"];
		if (params.count("preFilterCap")) this->preFilterCap = (int)params["preFilterCap"];
		if (params.count("uniquenessRatio")) this->uniquenessRatio = (int)params["uniquenessRatio"];
		if (params.count("speckleWindowSize")) this->speckleWindowSize = (int)params["speckleWindowSize"];
		if (params.count("speckleRange")) this->speckleRange = (int)params["speckleRange"];
		if (params.count("mode")) this->mode = (int)params["mode"];
	}

#ifdef YAML_PARAM
	void set(const YAML::Node &params) {
		if (params["min_disparity"]) this->min_disparity = params["min_disparity"].as<int>();
		if (params["num_disparities"]) this->num_disparities = params["num_disparities"].as<int>();
		if (params["blockSize"]) this->blockSize = params["blockSize"].as<int>();
		if (params["P1"]) this->P1 = params["P1"].as<int>();
		if (params["P2"]) this->P2 = params["P2"].as<int>();
		if (params["disp12MaxDiff"]) this->disp12MaxDiff = params["disp12MaxDiff"].as<int>();
		if (params["preFilterCap"]) this->preFilterCap = params["preFilterCap"].as<int>();
		if (params["uniquenessRatio"]) this->uniquenessRatio = params["uniquenessRatio"].as<int>();
		if (params["speckleWindowSize"]) this->speckleWindowSize = (int)params["speckleWindowSize"].as<int>();
		if (params["speckleRange"])	this->speckleRange = (int)params["speckleRange"].as<int>();
		if (params["mode"]) this->mode = (int)params["mode"].as<int>();
	}
#endif
	/**
	 * パラメータがアルゴリズムの許容範囲に収まっているかどうかをチェックする.
	 * @return 問題なければtrue, 問題あればfalse.
	 */
	bool check(void) const {
		if (num_disparities % 16 != 0) return false;
		if (blockSize % 2 != 1) return false;
		if (uniquenessRatio < 0 || uniquenessRatio > 100) return false;
		if (mode < 0 || mode > 3) return false;
		return true;
	}

	/**
	 * SGBM のパラメータ P1 の値を返す.
	 * @return P1の値
	 * @param [in] SGBM対象の画像のチャンネル数
	 */
	int getP1(const int channels = 1) {
		if (this->P1 < 0) {
			// 視差の滑らかさを制御するパラメータ(隣り合うピクセル間で視差が±1で変化した場合のペナルティ)
			return 8 * this->blockSize * this->blockSize * channels;
		}
		else return this->P1;
	}

	/**
	 * SGBM のパラメータ P2 の値を返す.
	 * @return P2の値
	 * @param [in] SGBM対象の画像のチャンネル数
	 */
	int getP2(const int channels = 1) {
		if (this->P2 < 0) {
			// 視差の滑らかさを制御するパラメータ(隣り合うピクセル間で視差が1よりも大きく変化した場合のペナルティ
			return 32 * this->blockSize * this->blockSize * channels;
		}
		else return this->P2;
	}
};
