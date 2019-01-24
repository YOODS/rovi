#pragma once

#include <opencv2/opencv.hpp>


class iFileStorage_error : public std::runtime_error {
public:
	iFileStorage_error(const std::string &what_arg) : std::runtime_error(what_arg) {}
	iFileStorage_error(const char *what_arg) : std::runtime_error(what_arg) {}
};




class CalibBoard {
public:
	CalibBoard() {
		// gamma補正値		
		para["gamma_correction"] = 1.0;

		// ヒストグラム均一化を行う(1), 行わない(0)
		para["do_equalize_hist"] = 0;

		// スムージングを行う(1), 行わない(0)		
		para["do_smoothing"] = 1;

		// 二値化タイプ(0: 通常二値化, 1: 判別分析二値化, 2: 適応二値化)		
		para["bin_type"] = 1;

        // 二値化閾値0(bin_type==0の場合閾値, bin_type==2の場合ブロックサイズ, その他の場合は使用しない)		
		para["bin_param0"] = 0;

		// 二値化閾値1(bin_type==2の場合平均値からのオフセット値, その他の場合は使われない)		
		para["bin_param1"] = 0;
	}

	virtual ~CalibBoard() {	}



protected:
	/**
	 * 初期化を行います.
	 * @return なし
	 * @param setting_filename 設定ファイル名(YAML形式)
	 */
	void init(const char *setting_filename = 0)
	{
		if (setting_filename == 0) return;

		cv::FileStorage fs(std::string(setting_filename), cv::FileStorage::READ);
		if (!fs.isOpened()) 
		{
			throw iFileStorage_error(setting_filename);
		}
		
		fs["gamma_correction"] >> para["gamma_correction"];
		fs["do_equalize_hist"] >> para["do_equalize_hist"];
		fs["do_smoothing"] >> para["do_smoothing"];
		fs["bin_type"] >> para["bin_type"];
		fs["bin_param0"] >> para["bin_param0"];
		fs["bin_param1"] >> para["bin_param1"];

		// ガンマテーブル初期化
		gamma_init(para["gamma_correction"]);
		fs.release();
	}

	
	/**
	 * キャリブレーションボードから特徴点を抽出するための前処理を実行する
	 * @return 前処理後の画像
	 * @param source 処理対象の画像
	 */
	cv::Mat preprocess(cv::Mat &source) 
	{
		cv::Mat target = source.clone();

		// ノイズ除去
		if (para["do_smoothing"] != 0) 
		{
			cv::Mat tmpim = source.clone();
			cv::medianBlur(tmpim, target, 3);
		}

		// gamma補正
		if (para["gamma_correction"] != 1.0) {
			cv::Mat tmpim = source.clone();
			gamma_correction(tmpim, target);
		}
				
		// ヒストグラム均一化
		if (para["do_equalize_hist"] != 0) 
		{
			cv::Mat tmpim = target.clone();
			cv::equalizeHist(tmpim, target);
		}

		// 二値化
		return binarize(target);
	}

	/**
	 * この設定をファイルに出力します.
	 * @return なし
	 * @param setting_filename 設定ファイル名
	 */
	void save_config(const char *setting_filename)
	{
		cv::FileStorage fs(std::string(setting_filename), cv::FileStorage::WRITE);
		if (fs.isOpened())
		{
			fs << "do_qualize_hist" << ((int) para["do_qualize_hist"]);
			fs << "do_smoothing" << ((int) para["do_smoothing"]);
			fs << "bin_type" << ((int) para["bin_type"]);
			fs << "bin_param0" << ((int) para["bin_param0"]);
			fs << "bin_param1" << ((int) para["bin_param1"]);
		}
		fs.release();
	}

private:
	/**
	 * 与えられた画像を二値化します.
	 * @return 二値化された画像
	 */
	cv::Mat binarize(cv::Mat &image) {
		cv::Mat retim;
		int bin_type = (int)para["bin_type"];
		switch (bin_type) {
		case 0:	// 通常の二値化
			cv::threshold(image, retim, para["bin_param0"], 255.0, cv::THRESH_BINARY);
			break;
		case 1:	// 判別分析二値化
			cv::threshold(image, retim, 0, 255, cv::THRESH_OTSU);
			break;
		case 2:	// 適応二値化
			cv::adaptiveThreshold(image, retim, 255.0, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY,
								  (int) para["bin_param0"], para["bin_param1"]);
			break;
		default:
			break;
		}
		return retim;
	}


	void gamma_init(const double gamma, const int range = 256);
	void gamma_correction(cv::Mat &source, cv::Mat &destim);
	
public:
	std::map<std::string, double> para;
	std::vector<int> gamma_table;
};



std::ostream& operator<<(std::ostream &os, const CalibBoard& obj);
