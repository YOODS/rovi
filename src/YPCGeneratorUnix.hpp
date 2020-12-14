#pragma once

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include "YPCGenerator.hpp"

class YPCGeneratorUnix : public YPCGenerator {
protected:
	PcGenMode pcgen_mode;

public:
	YPCGeneratorUnix() : pcgen_mode(PcGenMode::PCGEN_MULTI) {}
	virtual ~YPCGeneratorUnix() {
		if (stereo) stereo->destroy();
		stereo = 0;
		if (pcgen) pcgen->destroy();
		pcgen = 0;
	}

	/// 点群生成器を作成する.
	bool create_pcgen(const PcGenMode pcgen_mode);

	/// パラメータの初期化
	bool init(const char* cfgpath);
	//2020/08/26 modified by hato ---------- start ----------
	bool init(std::map<std::string,double> &paramMap);
	//2020/08/26 modified by hato ----------  end  ----------
	
	/// ステレオカメラ作成
	bool create_camera(const char* dirname);
	bool create_camera_raw(
		std::vector<double> &Kl, std::vector<double> &Kr,
		std::vector<double> &Dl, std::vector<double> &Dr,
		std::vector<double> &R, std::vector<double> &T);

	/// 入力ファイル名リスト作成
	std::vector<std::string> create_filelist(const char* dirname, const char* ext);

	/**
	  点群生成してPLYファイル保存を行います
	  @param [in] texture_cam テクスチャとして使用するカメラ番号(0: 左(従来通り), 1: 右)
	  @param [in] outpath PLYファイル名
	 */
	bool generate_pointcloud(const int texture_cam, const char* outpath);

	//2020/08/26 modified by hato ---------- start ----------
	///< 視差計算にかかった時間
	std::chrono::system_clock::duration get_elapsed_disparity(){
		return this->elapsed_preprocess;
	}
	
	///< 点群計算にかかった時間
	std::chrono::system_clock::duration get_elapsed_genpcloud(){
		return this->elapsed_genpcloud;
	}
	//2020/08/26 modified by hato ---------- end ----------
	
private:
	/// SGBMの設定値を読み込んで設定します.
	bool _init_SGBM(const YAML::Node& param);

	/// Grayコード位相シフト
	bool _init_PSFT(const YAML::Node& param);

	/// マルチ位相シフト
	bool _init_MPSFT(const YAML::Node& param);

	//2020/08/26 modified by hato ---------- start ----------
	/// SGBMの設定値を読み込んで設定します.
	bool _init_SGBM(std::map<std::string,double>& p);
	/// Grayコード位相シフト
	bool _init_PSFT(std::map<std::string,double>& p);
	/// マルチ位相シフト
	bool _init_MPSFT(std::map<std::string,double>& p);
	//2020/08/26 modified by hato ----------  end  ----------
	
	/// 経過時間表示
	void print_elapsed();
};
