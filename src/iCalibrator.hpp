#pragma once

#include <string>
#include "iCalibBoardRecognizer.hpp"


/// カメラキャリブパラメータ型
struct CameraCalibParam {
	double leng;	///< 焦点距離
	double cell;	///< センサーセルサイズ
	double F;				///< カメラパラメータF値
	int calibration_flags;	///< cv::cameraCalibrateのフラグ値

	CameraCalibParam() :
		leng(0.0), cell(0.0), F(0.0), calibration_flags(0) {}

	void setF() {
		if (leng == 0.0 || cell == 0.0) F = 0.0;
		else {
			F = leng / cell;
		}
	}

	/// パラメータ辞書から値をセットする. キーの名前はメンバ変数と同じ(だが、calibration_flagsだけはちょっと違う)
	void set(std::map<std::string, double> &params) {
		// フラグ値作成
		int cflags = 0;
		if (params.count("CV_CALIB_USE_INTRINSIC_GUESS") && params["CV_CALIB_USE_INTRINSIC_GUESS"] == 1.0)
			cflags += cv::CALIB_USE_INTRINSIC_GUESS;
		if (params.count("CV_CALIB_FIX_PRINCIPAL_POINT") && params["CV_CALIB_FIX_PRINCIPAL_POINT"] == 1.0)
			cflags += cv::CALIB_FIX_PRINCIPAL_POINT;
		if (params.count("CV_CALIB_FIX_ASPECT_RATIO") && params["CV_CALIB_FIX_ASPECT_RATIO"] == 1.0)
			cflags += cv::CALIB_FIX_ASPECT_RATIO;
		if (params.count("CV_CALIB_ZERO_TANGENT_DIST") && params["CV_CALIB_ZERO_TANGENT_DIST"] == 1.0)
			cflags += cv::CALIB_ZERO_TANGENT_DIST;
		if (params.count("CV_CALIB_FIX_K1") && params["CV_CALIB_FIX_K1"] == 1.0)
			cflags += cv::CALIB_FIX_K1;
		if (params.count("CV_CALIB_FIX_K2") && params["CV_CALIB_FIX_K2"] == 1.0)
			cflags += cv::CALIB_FIX_K2;
		if (params.count("CV_CALIB_FIX_K3") && params["CV_CALIB_FIX_K3"] == 1.0)
			cflags += cv::CALIB_FIX_K3;
		if (params.count("CV_CALIB_FIX_K4") && params["CV_CALIB_FIX_K4"] == 1.0)
			cflags += cv::CALIB_FIX_K4;
		if (params.count("CV_CALIB_FIX_K5") && params["CV_CALIB_FIX_K5"] == 1.0)
			cflags += cv::CALIB_FIX_K5;
		if (params.count("CV_CALIB_FIX_K6") && params["CV_CALIB_FIX_K6"] == 1.0)
			cflags += cv::CALIB_FIX_K6;
		if (params.count("CV_CALIB_RATIONAL_MODEL") && params["CV_CALIB_RATIONAL_MODEL"] == 1.0)
			cflags += cv::CALIB_RATIONAL_MODEL;
		this->calibration_flags = cflags;

		if (params.count("cell")) this->cell = params["cell"];
		if (params.count("leng")) this->leng = params["leng"];
		this->setF();
	}
};


class iCameraCalibrator {
public:
	/**
	 * デストラクタ呼び出し
	 */
	virtual void destroy() = 0;

	/**
	 * パラメータを設定します.
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] prcp マーカ検出のための前処理用パラメータ
	 * @param [in] mrkp 円マーカ検出のためのパラメータ
	 * @param [in] brdp キャリブボードのパラメータ
	 */
	virtual bool set_parameters(PreProcParam &prcp, CircleMarkerParam &mrkp, CalibBoardParam &brdp) = 0;

	/**
	 * キャリブレーションターゲット画像を一枚追加します.
	 * @return 0が成功. -1がマーカ検出器が作成されていない. その他はマーカ検出状態値.
	 * @param [in] image 画像左上端アドレス
	 * @param [in] width 画像横幅
	 * @param [in] height 画像縦幅
	 * @param [in] step 画像バッファの水平方向のバイト数
	 * @param [in] show_result マーカ検出結果画像を画面に表示する(true)か否(false)か
	 * @param [in] filename マーカ検出結果画像保存先ファイル名(string()を与えた場合は保存しない)
	 */
	virtual int put_image(unsigned char *image, const int width, const int height, const size_t step, const bool show_result = true, const std::string filename = std::string()) = 0;

	/**
	 * キャリブレーションターゲット画像をファイルから読み込んで一枚追加します.
	 * @return 0が成功. -1がマーカ検出器が作成されていない. -2が画像が読み込めない. その他はマーカ検出状態値.
	 * @param [in] i_filename 入力画像ファイル名
	 * @param [in] show_result マーカ検出結果画像を画面に表示する(true)か否(false)か
	 * @param [in] o_filename マーカ検出結果画像保存先ファイル名(string()を与えた場合は保存しない)
	 * @warning 連結画像には未対応です.
	 */
	virtual int put_image(const std::string i_filename, const bool show_result = true, const std::string o_filename = std::string()) = 0;

	/**
	 * キャリブレーションを実行します.
	 * @return キャリブレーションに成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] camp キャリブレーションパラメータ
	 * @param [out] reprojerr 再投影誤差
	 * @param [out] camerr カメラ誤差
	 */
	virtual bool do_calibrate(CameraCalibParam &camp, std::vector<double>* reprojerr = 0, double *camerr = 0) = 0;

	/**
	 * キャリブレーション結果をファイルに保存します.
	 * @return 保存に成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] filename 保存先ファイル名.
	 * @param [in] cam_name カメラ名(パラメータファイルを読み込んでカメラを作成するときに、同じカメラ名でないと正しく読み込めません.)
	 * @warning ファイル名の拡張子を".xml"とすればXML形式で、".yaml"とすればYAML形式で保存されます.
	 */
	virtual bool save_calibfile(const std::string filename, const std::string cam_name = std::string()) = 0;
};


/// レクティファイパラメータ型
struct RectifyParam {
	double alpha;	///< スケーリングパラメータ
	int flags;		///< レクティファイフラグ
	int new_width;	///< レクティファイ後の画像横幅
	int new_height;	///< レクティファイ後の画像縦幅
	int nopL;		///< 左カメラ用背景画素値
	int nopR;		///< 右カメラ用背景画素値

	RectifyParam() :
		alpha(-1.0),	///< スケーリングパラメータ.-1以外の値であれば0～1の間の値.0だと有効な範囲のみ画像に残るように拡大され、1だと元の画像が全て収まるように縮小される
		flags(0),		///< 処理フラグ.主点を左右でずらす
		new_width(0),	///< レクティファイ後の画像横幅(0の場合は入力と同じ)
		new_height(0),	///< レクティファイ後の画像縦幅(0の場合は入力と同じ)
		nopL(0),		///< 左画像の無効な画素に対して与えられる値(0～255)
		nopR(0) 		///< 右画像の無効な画素に対して与えられる値(0～255)
	{}

	/// パラメータ辞書から値をセットする. キーの名前はメンバ変数と同じ
	void set(std::map<std::string, double> &params) {
		if (params.count("alpha")) this->alpha = params["alpha"];
		if (params.count("flags")) this->flags = (params["flags"] == 0.0) ? 0 : cv::CALIB_ZERO_DISPARITY;
		if (params.count("new_width")) this->new_width = (params["new_width"] <= 0) ? 0 : (int)params["new_width"];
		if (params.count("new_height")) this->new_height = (params["new_height"] <= 0) ? 0 : (int)params["new_height"];
		if (params.count("nopL")) this->nopL = (params["nopL"] <= 0) ? 0 : ((params["nopL"] >= 255) ? 255 : (int)params["nopL"]);
		if (params.count("nopR")) this->nopL = (params["nopR"] <= 0) ? 0 : ((params["nopR"] >= 255) ? 255 : (int)params["nopR"]);
	}
};


class iStereoCalibrator {
public:
	/**
	 * デストラクタ呼び出し
	 */
	virtual void destroy() = 0;

	/**
	 * パラメータを設定します.
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] prcp マーカ検出のための前処理用パラメータ
	 * @param [in] mrkp 円マーカ検出のためのパラメータ
	 * @param [in] brdp キャリブボードのパラメータ
	 */
	virtual bool set_parameters(PreProcParam &prcp, CircleMarkerParam &mrkp, CalibBoardParam &brdp) = 0;

	/**
	 * キャリブレーションターゲット画像を一枚追加します.
	 * @return 0が成功.-1がマーカ検出器が作成されていない. -2がカメラ番号が不正. その他はマーカ検出状態値.
	 * @param [in] cid カメラ番号
	 * @param [in] image 画像左上端アドレス
	 * @param [in] width 画像横幅
	 * @param [in] height 画像縦幅
	 * @param [in] step 画像バッファの水平方向のバイト数
	 * @param [in] show_result マーカ検出結果画像を画面に表示する(true)か否(false)か
	 * @param [in] filename マーカ検出結果画像保存先ファイル名(string()を与えた場合は保存しない)
	 * @warning 追加される画像は左右で順番を変えないでください. (左カメラn番目の画像と右カメラn番目の画像は必ず同じシーンを
	 * 左右カメラで撮影したものとしてください).
	 */
	virtual int put_image(const int cid, unsigned char *image, const int width, const int height, const size_t step, const bool show_result = true, const std::string filename = std::string()) = 0;

	/**
	 * キャリブレーションターゲット画像をファイルから読み込んで一枚追加します.
	 * @return 0が成功.-1がマーカ検出器が作成されていない. -2がカメラ番号が不正. -3が画像が読み込めない. その他はマーカ検出状態値.
	 * @param [in] cid カメラ番号
	 * @param [in] i_filename 入力画像ファイル名
	 * @param [in] show_result マーカ検出結果画像を画面に表示する(true)か否(false)か
	 * @param [in] o_filename マーカ検出結果画像保存先ファイル名(string()を与えた場合は保存しない)
	 * @warning 追加される画像は左右で順番を変えないでください. (左カメラn番目の画像と右カメラn番目の画像は必ず同じシーンを
	 * 左右カメラで撮影したものとしてください).
	 * @warning 分離画像用です.
	 */
	virtual int put_image(const int cid, const std::string i_filename, const bool show_result = true, const std::string filename = std::string()) = 0;

	/**
	 * キャリブレーションターゲット画像(左右連結画像)をファイルから読み込んで追加します.
	 * @return 0が成功.-1がマーカ検出器が作成されていない. -2が画像が読み込めない. その他はマーカ検出状態値(上4bit左カメラ, 下4bit右カメラ).
	 * @param [in] i_filename 入力画像ファイル名
	 * @param [in] show_result マーカ検出結果画像を画面に表示する(true)か否(false)か
	 * @param [in] o_filename マーカ検出結果画像保存先ファイル名(string()を与えた場合は保存しない)
	 * @warning 連結画像(横連結)画像用です.
	 */
	virtual int put_image(const std::string i_filename, const bool show_result = true, const std::string filename = std::string()) = 0;

	/**
	 * キャリブレーションを実行します.
	 * @return キャリブレーションに成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] camp キャリブレーションパラメータ
	 * @param [out] reprojerr 再投影誤差
	 * @param [out] camerr カメラ誤差
	 */
	virtual bool do_calibrate(CameraCalibParam &camp, std::vector<std::vector<double>>* reprojerr = 0, double *camerr = 0) = 0;

	/**
	 * キャリブレーション結果(歪み有カメラのカメラパラメータ)をファイルに保存します.
	 * @return 保存に成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] cam0_filename 基準カメラ(左カメラ)のパラメータ保存先ファイル名.
	 * @param [in] cam1_filename 補助カメラ(右カメラ)のパラメータ保存先ファイル名.
	 * @warning ファイル名の拡張子を".xml"とすればXML形式で、".yaml"とすればYAML形式で保存されます.
	 */
	virtual bool save_calibfile(const std::string cam0_filename, const std::string cam1_filename) = 0;

	/**
	 * レクティファイを実行し、ステレオカメラを作成します.
	 * @return 作成に成功した場合は基線長が、失敗した場合は-1が返ります.
	 * @param [in] rp レクティファイパラメータ
	 * @param [out] roiL 左カメラの元画像における有効範囲(x, y, w, h)を格納する配列(4個分のintが必要)
	 * @param [out] roiR 右カメラの元画像における有効範囲(x, y, w, h)を格納する配列(4個分のintが必要)
	 */
	virtual double create_stereo_camera(RectifyParam &rp, int *roiL = 0, int *roiR = 0) = 0;		

	/**
	 * ボード座標系への変換行列を作成します.
	 * @return 成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] imageL 左カメラ参照画像
	 * @param [in] imageR 右カメラ参照画像
	 */
	virtual bool create_stereo_brdcrd(unsigned char *imageL, const size_t stepL, unsigned char *imageR, const size_t stepR) = 0;

	/**
	 * ボード座標系への変換行列を作成します.
	 * @return 成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] filenameL 左カメラ参照画像ファイル名
	 * @param [in] filenameR 右カメラ参照画像ファイル名
	 * @warning 分割画像用です.
	 */
	virtual bool create_stereo_brdcrd(const std::string filenameL, const std::string filenameR) = 0;

	/**
	 * ボード座標系への変換行列を作成します.
	 * @return 成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] filename 左右カメラ参照画像ファイル名
	 * @warning 連結画像用です.
	 */
	virtual bool create_stereo_brdcrd(const std::string filename) = 0;

	/**
	 * 作成されたステレオカメラパラメータをファイルに保存します.
	 * @return 保存に成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] filename ステレオカメラパラメータ保存先ファイル名
	 * @param [in] lname 左カメラのレクティファイマップ保存先ファイル名
	 * @param [in] rname 右カメラのレクティファイマップ保存先ファイル名
	 * @warning ファイル名の拡張子を".xml"とすればXML形式で、".yaml"とすればYAML形式で保存されます.
	 * @warning lname, rnameを指定しない場合、レクティファイマップは保存されません.
	 */
	virtual bool save_stereofile(const std::string filename, 
		const std::string lname = std::string(), const std::string rname = std::string()) = 0;

	/**
	 * HMat形式でステレオカメラパラメータをファイルに保存します.
	 * @return 保存に成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] hmat0_name 左カメラPPM(Hmat)用のファイル名
	 * @param [in] hmat1_name 右カメラPPM(Hmat)用のファイル名
	 * @param [in] rectmap_name レクティファイテーブル用のファイル名
	 * @param [in] isbrd ボード座標系に変換してから出力するか否か
	 */
	virtual bool save_stereohmat(const std::string hmat0_name, const std::string hmat1_name, const std::string rectmap_name, const bool isbrd = false) = 0;

	/**
	 * ボード座標系への変換行列をファイルに保存します.
	 * @return 保存に成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] filename 保存先ファイル名
	 */
	virtual bool save_boardRT(const std::string filename) = 0;
};

#ifdef _WINDOWS
#ifdef _WINDLL
#define EXPORT_CALIB __declspec(dllexport)
#else
#define EXPORT_CALIB __declspec(dllimport)
#endif
#else
#define EXPORT_CALIB
#endif


/**
 * 単体カメラキャリブレーションクラスインスタンス作成関数
 * @return クラスインスタンスのアドレス
 */
extern "C" EXPORT_CALIB iCameraCalibrator* CreateCameraCalibrator();
typedef iCameraCalibrator *(*pCreateCameraCalibrator)();

/**
 * ステレオカメラキャリブレーションクラスインスタンス作成関数
 * @return クラスインスタンスのアドレス
 */
extern "C" EXPORT_CALIB iStereoCalibrator* CreateStereoCalibrator();
typedef iStereoCalibrator *(*pCreateStereoCalibrator)();