#include <iostream>
#include <dirent.h>
#include <algorithm>
#include "YPCGeneratorUnix.hpp"
#include "ParamSGBM.hpp"
#include "ParamPSFT.hpp"
#include "ParamMPSFT.hpp"
#include "iPointCloudGenerator.hpp"
#include "iStereoCamera.hpp"

//2020/08/26 modified by hato ---------- start ----------
#include <sys/stat.h>
//2020/08/26 modified by hato ----------  end  ----------


//2020/08/31 modified by hato ---------- start ----------
//#define DETAIL_DEBUG_LOG
//2020/08/31 modified by hato ----------  end  ----------


bool YPCGeneratorUnix::create_pcgen(const PcGenMode pcgen_mode_)
{
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	std::cout << "YPCGeneratorUnix::create_pcgen() " << pcgen_mode_ << std::endl;
#endif
	//2020/08/31 modified by hato ----------  end  ----------
	if (this->pcgen) this->pcgen->destroy();
	this->pcgen_mode = pcgen_mode_;	
	this->pcgen = CreatePointCloudGenerator(pcgen_mode_);
	return (this->pcgen) ? true : false;
}


//2020/08/26 modified by hato ---------- start ----------
#ifdef YAML_PARAM
//2020/08/26 modified by hato ----------  end  ----------

bool YPCGeneratorUnix::init(const char* cfgpath)
{
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	std::cout << "YPCGeneratorUnix::init()\n";
#endif
	//2020/08/31 modified by hato ----------  end  ----------
	if (!this->pcgen) {
		std::cerr << "no point cloud generator\n";
		return false;
	}
	if (!cfgpath) {
		std::cerr << "no inipath\n";
		return false;
	}

	YAML::Node params;
	try {
		params = YAML::LoadFile(std::string(cfgpath));
	}
	catch (YAML::Exception&) {
		std::cerr << "load parameter failure\n";
		return false;
	}
	
	// 読み込んだパラメータを表示
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	std::cerr << params << std::endl;
#endif
	//2020/08/31 modified by hato ----------  end  ----------
	
	// 共通パラメータ設定
	if (params["image_width"]) set_camera_cols(params["image_width"].as<int>());
	if (params["image_height"]) set_camera_rows(params["image_height"].as<int>());
	if (params["method3d"]) this->method3d = (iPointCloudGenerator::Method3D)(params["method3d"].as<int>());
	if (params["camera_type"]) this->camtype = (CamParamType)(params["camera_type"].as<int>());

	if (this->pcgen_mode == PcGenMode::PCGEN_SGBM) {
		// SGBMの場合
		return _init_SGBM(params["SGBM"]);
	}
	else if (this->pcgen_mode == PcGenMode::PCGEN_GRAYPS4) {
		// Grayコード位相シフト
		return _init_PSFT(params["PSFT"]);
	}
	else if (this->pcgen_mode == PcGenMode::PCGEN_MULTI) {
		// マルチ位相シフト
		return _init_MPSFT(params["MPSFT"]);
	}
	else return false;
}

//2020/08/26 modified by hato ----------  end  ----------
#endif
//2020/08/26 modified by hato ---------- start ----------

bool YPCGeneratorUnix::init(std::map<std::string,double> &params){
	
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	std::cout << "YPCGeneratorUnix::init()\n";
#endif
	//2020/08/31 modified by hato ----------  ebd  ----------
	if (!this->pcgen) {
		std::cerr << "no point cloud generator\n";
		return false;
	}
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	// 読み込んだパラメータを表示
	for (std::map<std::string, double>::const_iterator i = params.begin(); i != params.end(); ++i) {
        std::cout << i->first << " => " << i->second << std::endl;
    }
#endif
	//2020/08/31 modified by hato ----------  ebd  ----------
	
	// 共通パラメータ設定
	if ( params.count("image_width") ) set_camera_cols(params.at("image_width"));
	if ( params.count("image_height") ) set_camera_rows((int)params.at("image_height"));
	if ( params.count("method3d") ) this->method3d = (iPointCloudGenerator::Method3D)((int)params.at("method3d"));
	if ( params.count("camera_type") ) this->camtype = (CamParamType)((int)params.at("camera_type"));
	
	if (this->pcgen_mode == PcGenMode::PCGEN_SGBM) {
		// SGBMの場合
		return _init_SGBM(params);
	}
	else if (this->pcgen_mode == PcGenMode::PCGEN_GRAYPS4) {
		// Grayコード位相シフト
		return _init_PSFT(params);
	}
	else if (this->pcgen_mode == PcGenMode::PCGEN_MULTI) {
		// マルチ位相シフト
		return _init_MPSFT(params);
	}
	else return false;
	
}

//2020/08/26 modified by hato ----------  end  ----------


bool YPCGeneratorUnix::create_camera(const char* dirname)
{
	
	if (!pcgen) return false;
	if (stereo) {
		stereo->destroy();
		stereo = 0;
	}

	std::string dirnm(dirname);

	//std::cerr << "YPCGeneratorUnix: create stereo camera from\n";
	std::vector<std::string> filenames;
	if (this->camtype == CamParamType::HMat) {
		std::string name = dirnm + "/hmat0.dat";
		filenames.push_back(name);
		//std::cerr << "\t" << name.c_str() << "\n";

		name = dirnm + "/hmat1.dat";
		filenames.push_back(name);
		//std::cerr << "\t" << name.c_str() << "\n";

		name = dirnm + "/rect.param";
		filenames.push_back(name);
		//std::cerr << "\t" << name.c_str() << "\n";
	}
	else if (this->camtype == CamParamType::CamN) {
		std::string name = dirnm + "/cam0_param.yaml";
		filenames.push_back(name);
		//std::cerr << "\t" << name.c_str() << "\n";

		name = dirnm + "/cam1_param.yaml";
		filenames.push_back(name);
		//std::cerr << "\t" << name.c_str() << "\n";
	}
	else {
		std::cerr << "Camera Type Error\n";
		return false;
	}
	
	struct stat buffer;   
	bool allExists=true;
	for(const std::string &filename:filenames){
		if( stat (filename.c_str(), &buffer) != 0){
			std::cerr << "file not found. path=" << filename.c_str() << "\n";
			allExists=false;
			break;
		}
	}
	if ( ! allExists ) { return false; }
	
	//fprintf(stderr,"camera:(input)  width=%d height=%d\n",this->settings.input_cols,this->settings.input_rows);
	//fprintf(stderr,"camera:(output) width=%d height=%d\n",this->settings.output_cols,this->settings.output_rows);
	
	if (!(this->stereo = CreateStereoCamera(this->camtype, filenames, &this->settings))) return false;

	// 点群生成器にステレオカメラを渡す
	this->pcgen->init(this->stereo);
	return true;
}

bool YPCGeneratorUnix::create_camera_raw(
		std::vector<double> &Kl, std::vector<double> &Kr,
		std::vector<double> &Dl, std::vector<double> &Dr,
		std::vector<double> &R, std::vector<double> &T)
{
	if (!(this->stereo = CreateStereoCameraFromRaw(Kl, Kr, Dl, Dr, R, T, &this->settings))) return false;
	
	// 点群生成器にステレオカメラを渡す
	this->pcgen->init(this->stereo);
	return true;	
}



std::vector<std::string>
YPCGeneratorUnix::create_filelist(const char* dirname, const char* ext)
{
	DIR *dp = opendir(dirname);
	if (dp == NULL) {
		return std::vector<std::string>();
	}

	std::vector<std::string> filelist;
	std::string path(dirname);
	
	// ディレクトリ内のファイル一覧を作成
	struct dirent *dent;
	while ((dent = readdir(dp)) != NULL) {
		std::string name(dent->d_name);
		if (name.find(ext) != std::string::npos) {
			// 指定された拡張子を含んでいたら名前を取っておく
			std::string name_ = path + "/" + name;
			filelist.push_back(name_);
		}
	}
	closedir(dp);

	if (filelist.size() == 0) {
		std::cerr << "YPCGeneratorUnix: images not found in " << dirname << std::endl;
		return std::vector<std::string>();		
	}
	
	
	// アルファベット順に並べ替える
	std::sort(filelist.begin(), filelist.end());
	return filelist;
}

bool YPCGeneratorUnix::generate_pointcloud(std::vector<std::string>& filelist, const char* outpath, const bool is_interpo)
{
	std::cerr << "YPCGeneratorUnix: generate point cloud\n";
	PLYSaver saver(outpath);
	int num = YPCGenerator::generate_pointcloud(filelist, is_interpo, &saver);
	std::cerr << "n_points = " << num << std::endl;
	std::cerr << saver.get_filename() << " : ply file save " << (saver.is_ok() ? "success" : "failure") << std::endl;
	return (num == 0) ? false : true;
}

//2020/08/25 modified by hato ---------- start ----------

int YPCGeneratorUnix::generate_pointcloud(std::vector<unsigned char*>& buffers, const bool is_interpo,PointCloudCallback *callback){
	//std::cerr << "YPCGeneratorUnix: generate point cloud \n";	
	const int num = YPCGenerator::generate_pointcloud(buffers, is_interpo, callback );	
	//std::cerr << "n_points = " << num << std::endl;
	return num;
}

//2020/08/25 modified by hato ----------  end  ----------

//2020/08/25 modified by hato ---------- start ----------
#ifdef YAML_PARAM
//2020/08/25 modified by hato ----------  end  ----------

bool YPCGeneratorUnix::_init_SGBM(const YAML::Node& p)
{
	
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	std::cerr << "YPCGeneratorUnix::_init_SGBM()\n";
#endif
	//2020/08/31 modified by hato ----------  end  ----------
	SGBMParameter param;
	param.set(p);
	if (!pcgen->setparams(&param)) {
		// パラメータが何か間違っている
		std::cerr << "params value failure\n";
		return false;
	}

	// 読み込んだパラメータを表示
	
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	std::cerr << "\tmin_disparity    : " << param.min_disparity << std::endl;
	std::cerr << "\tnum_disparities  : " << param.num_disparities << std::endl;
	std::cerr << "\tblockSize        : " << param.blockSize << std::endl;
	std::cerr << "\tdisp12MaxDiff    : " << param.disp12MaxDiff << std::endl;
	std::cerr << "\tpreFilterCap     : " << param.preFilterCap << std::endl;
	std::cerr << "\tuniquenessRatio  : " << param.uniquenessRatio << std::endl;
	std::cerr << "\tspeckleWindowSize: " << param.speckleWindowSize << std::endl;
	std::cerr << "\tspeckleRange     : " << param.speckleRange << std::endl;
	std::cerr << "\tmode             : " << param.mode << std::endl;
#endif
	//2020/08/31 modified by hato ----------  end  ----------
	return true;
}

bool YPCGeneratorUnix::_init_PSFT(const YAML::Node& p)
{
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	std::cerr << "YPCGeneratorUnix::_init_PSFT()\n";
#endif
	//2020/08/31 modified by hato ----------  end  ----------
	
	GPhaseDecodeParameter param;
	param.set(p);
	if (!pcgen->setparams(&param)) {
		// パラメータが何か間違っている
		std::cerr << "params value failure\n";
		return false;
	}
	// パラメータ表示
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	std::cerr << "\tbw_diff    : " << param.bw_diff << std::endl;
	std::cerr << "\tbrightness : " << param.brightness << std::endl;
	std::cerr << "\tdarkness   : " << param.darkness << std::endl;

	std::cerr << "\tphase_wd_min     : " << param.phase_wd_min << std::endl;
	std::cerr << "\tphase_wd_thr     : " << param.phase_wd_thr << std::endl;
	std::cerr << "\tgcode_variation  : " << param.gcode_variation << std::endl;

	std::cerr << "\tmax_parallax : " << param.max_parallax << std::endl;
	std::cerr << "\tmin_parallax : " << param.min_parallax << std::endl;
	std::cerr << "\tmax_ph_diff  : " << param.max_ph_diff << std::endl;

	std::cerr << "\tls_points    : " << param.ls_points << std::endl;
#endif
	//2020/08/31 modified by hato ----------  end  ----------
	return true;
}

bool YPCGeneratorUnix::_init_MPSFT(const YAML::Node& p)
{
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	std::cerr << "YPCGeneratorUnix::_init_MPSFT()\n";
#endif
	MPhaseDecodeParameter param;
	param.set(p);
	
	if (!pcgen->setparams((void*)&param)) {
		// パラメータが何か間違っている
		std::cerr << "params value failure\n";
		return false;
	}
	
	// パラメータ表示
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	std::cerr << "\tbw_diff    : " << param.bw_diff << std::endl;
	std::cerr << "\tbrightness : " << param.brightness << std::endl;
	std::cerr << "\tdarkness   : " << param.darkness << std::endl;

	std::cerr << "\tn_phaseshift : " << param.n_phaseshift << std::endl;
	std::cerr << "\tn_periods    : " << param.n_periods << std::endl;
	std::cerr << "\tpreiod       : " << param.period[0] << ", " << param.period[1] << ", " << param.period[2] << std::endl;

	std::cerr << "\tmax_parallax : " << param.max_parallax << std::endl;
	std::cerr << "\tmin_parallax : " << param.min_parallax << std::endl;
	std::cerr << "\tmax_ph_diff  : " << param.max_ph_diff << std::endl;

	std::cerr << "\tls_points    : " << param.ls_points << std::endl;
#endif
	//2020/08/31 modified by hato ----------  end  ----------
	return true;
}

//2020/08/25 modified by hato ---------- start ----------
#endif
//2020/08/25 modified by hato ----------  end  ----------


bool YPCGeneratorUnix::_init_SGBM(std::map<std::string,double>& p)
{
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	std::cerr << "YPCGeneratorUnix::_init_SGBM()\n";
#endif
//2020/08/25 modified by hato ----------  end  ----------
	
	SGBMParameter param;
	param.set(p);
	if (!pcgen->setparams(&param)) {
		// パラメータが何か間違っている
		std::cerr << "params value failure\n";
		return false;
	}
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	// 読み込んだパラメータを表示
	std::cerr << "\tmin_disparity    : " << param.min_disparity << std::endl;
	std::cerr << "\tnum_disparities  : " << param.num_disparities << std::endl;
	std::cerr << "\tblockSize        : " << param.blockSize << std::endl;
	std::cerr << "\tdisp12MaxDiff    : " << param.disp12MaxDiff << std::endl;
	std::cerr << "\tpreFilterCap     : " << param.preFilterCap << std::endl;
	std::cerr << "\tuniquenessRatio  : " << param.uniquenessRatio << std::endl;
	std::cerr << "\tspeckleWindowSize: " << param.speckleWindowSize << std::endl;
	std::cerr << "\tspeckleRange     : " << param.speckleRange << std::endl;
	std::cerr << "\tmode             : " << param.mode << std::endl;
#endif
//2020/08/25 modified by hato ----------  end  ----------
	return true;
}

bool YPCGeneratorUnix::_init_PSFT(std::map<std::string,double>& p)
{
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	std::cerr << "YPCGeneratorUnix::_init_PSFT()\n";
#endif
//2020/08/25 modified by hato ----------  end  ----------
	GPhaseDecodeParameter param;
	param.set(p);
	if (!pcgen->setparams(&param)) {
		// パラメータが何か間違っている
		std::cerr << "params value failure\n";
		return false;
	}
	// パラメータ表示
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	std::cerr << "\tbw_diff    : " << param.bw_diff << std::endl;
	std::cerr << "\tbrightness : " << param.brightness << std::endl;
	std::cerr << "\tdarkness   : " << param.darkness << std::endl;

	std::cerr << "\tphase_wd_min     : " << param.phase_wd_min << std::endl;
	std::cerr << "\tphase_wd_thr     : " << param.phase_wd_thr << std::endl;
	std::cerr << "\tgcode_variation  : " << param.gcode_variation << std::endl;

	std::cerr << "\tmax_parallax : " << param.max_parallax << std::endl;
	std::cerr << "\tmin_parallax : " << param.min_parallax << std::endl;
	std::cerr << "\tmax_ph_diff  : " << param.max_ph_diff << std::endl;

	std::cerr << "\tls_points    : " << param.ls_points << std::endl;
#endif
//2020/08/25 modified by hato ----------  end  ----------
	
	return true;
}

bool YPCGeneratorUnix::_init_MPSFT(std::map<std::string,double>& p)
{
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	std::cerr << "YPCGeneratorUnix::_init_MPSFT()\n";
#endif
//2020/08/25 modified by hato ----------  end  ----------
	MPhaseDecodeParameter param;
	param.set(p);
	
	if (!pcgen->setparams((void*)&param)) {
		// パラメータが何か間違っている
		std::cerr << "params value failure\n";
		return false;
	}
	
	// パラメータ表示
	//2020/08/31 modified by hato ---------- start ----------
#ifdef DETAIL_DEBUG_LOG
	std::cerr << "\tbw_diff    : " << param.bw_diff << std::endl;
	std::cerr << "\tbrightness : " << param.brightness << std::endl;
	std::cerr << "\tdarkness   : " << param.darkness << std::endl;

	std::cerr << "\tn_phaseshift : " << param.n_phaseshift << std::endl;
	std::cerr << "\tn_periods    : " << param.n_periods << std::endl;
	std::cerr << "\tpreiod       : " << param.period[0] << ", " << param.period[1] << ", " << param.period[2] << std::endl;

	std::cerr << "\tmax_parallax : " << param.max_parallax << std::endl;
	std::cerr << "\tmin_parallax : " << param.min_parallax << std::endl;
	std::cerr << "\tmax_ph_diff  : " << param.max_ph_diff << std::endl;

	std::cerr << "\tls_points    : " << param.ls_points << std::endl;
#endif
//2020/08/25 modified by hato ----------  end  ----------
	return true;
}

