#pragma once

#include <vector>
#include <string>

//2020/08/25 modified by hato ---------- start ----------
#ifdef YAML_PARAM
//2020/08/25 modified by hato ----------  end  ----------
#include <yaml-cpp/yaml.h>
//2020/08/25 modified by hato ---------- start ----------
#endif
//2020/08/25 modified by hato ----------  end  ----------

//2020/08/26 modified by hato ---------- start ----------
#include <map>
//2020/08/26 modified by hato ----------  end ----------

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
	//2020/08/26 modified by hato ---------- start ----------
#ifdef YAML_PARAM
	bool init(const char* cfgpath);
#endif
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
	  @param [in] filelist 入力画像ファイル名リスト
	  @param [in] outpath PLYファイル名
	  @param [in] is_interpo 補間を行うか否か
	 */
	bool generate_pointcloud(std::vector<std::string> &filelist, const char* outpath, const bool is_interpo);
	
//2020/08/25 modified by hato ---------- start ----------
	/**
	  点群生成してPLYファイル保存を行います
	  @param [in] cam_imgs 左右カメラ画像
	  @param [in] outpath PLYファイル名
	  @param [in] is_interpo 補間を行うか否か
	 */
	int generate_pointcloud(std::vector<unsigned char*>& cam_imgs, const bool is_interpo,PointCloudCallback*callback);

//2020/08/25 modified by hato ----------  end  ----------

//2020/08/26 modified by hato ---------- start ----------
	///< 視差計算にかかった時間
	std::chrono::system_clock::duration get_elapsed_disparity(){
		return this->elapsed_disparity;
	}
	
	///< 点群計算にかかった時間
	std::chrono::system_clock::duration get_elapsed_genpcloud(){
		return this->elapsed_genpcloud;
	}
//2020/08/26 modified by hato ---------- end ----------
	
private:
//2020/08/25 modified by hato ---------- start ----------
#ifdef YAML_PARAM
//2020/08/25 modified by hato ----------  end  ----------
	/// SGBMの設定値を読み込んで設定します.
	bool _init_SGBM(const YAML::Node& param);

	// Grayコード位相シフト
	bool _init_PSFT(const YAML::Node& param);

	// マルチ位相シフト
	bool _init_MPSFT(const YAML::Node& param);
//2020/08/25 modified by hato ---------- start ----------
#endif
//2020/08/25 modified by hato ----------  end  ----------

//2020/08/26 modified by hato ---------- start ----------
	bool _init_SGBM(std::map<std::string,double>& p);
	bool _init_PSFT(std::map<std::string,double>& p);
	bool _init_MPSFT(std::map<std::string,double>& p);
//2020/08/26 modified by hato ----------  end  ----------
};
