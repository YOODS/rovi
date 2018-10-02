#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include "PointCloud.h"


/// レクティファイマップデータのアライメント
#define MAP_DATA_ALIGNMENT	(8)


/**
 * ピンホールカメラモデル
 */
class PinHoleCamera {
public:
	std::string name;		///< カメラ名
	cv::Size size;			///< 画像サイズ	

	cv::Mat_<double> K;		///< 内部パラメータ行列
	cv::Mat_<double> R;		///< カメラ回転行列
	cv::Mat_<double> T;		///< カメラ並進ベクトル
	cv::Mat_<double> fP;	///< 順方向透視投影行列
	cv::Mat_<double> rP;	///< 逆方向透視投影行列

public:
	/**
	 * コンストラクタ
	 * @param[in] _size 画像の大きさ
	 * @param[in] _name このカメラの名前
	 */
	PinHoleCamera(cv::Size _size = cv::Size(0, 0), std::string _name = "camph");

	/// デストラクタ
	virtual ~PinHoleCamera() {}

	/**
	 * このカメラの画像の大きさと名前を設定します.
	 * @param[in] _size 画像の大きさ
	 * @param[in] _name このカメラの名前
	 */
	void set(cv::Size _size, std::string _name) {
		size = _size;
		name = _name;
	}

	/**
	 * 全てのカメラパラメータを保存するファイル名を返します.
	 * @return ファイル名
	 */
	std::string create_param_filename(void) { return name + "_param.yaml"; }

	/**
	 * カメラパラメータをファイルストレージに書き出す
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] fs ファイルストレージへのポインタ.0が指定された場合は、create_param_filename()で得られるファイル名で出力する。
	 */
	bool save(cv::FileStorage *_fs = 0);

	/**
	 * PPMを指定されたファイルストリームに書き出す.(旧型式用)
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] ofs 出力用ファイルストリーム
	 */
	bool save_hmat(std::ofstream &ofs);

	/**
	 * カメラパラメータをファイルストレージから読み込む。
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] fs ファイルストレージへのポインタ.0が指定された場合は、create_param_filename()で得られるファイル名で出力する。
	 */
	bool load(cv::FileStorage *_fs = 0);

	/**
	 * 指定されたファイルストリームからPPMを読み込む.(旧型式用)
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] ifs 入力用ファイルストリーム
	 */
	bool load_hmat(std::ifstream &ifs);

	/**
	 * 順方向透視投影行列を分解して、K,R,Tを求めます。
	 * @return なし.
	 */
	void decompose();

	/**
	 * K,R,Tから透視投影行列を求めます.
	 * @return なし.
	 */
	void compose();

	/**
	 * このカメラの透視投影行列を与えられた行列に変更します.
	 * @return なし.
	 * @param[in] P 新しい透視投影行列
	 * @warning P は3x4行列であることを前提にしています.それ以上の大きさであれば、3x4にしてから渡してください.
	 */
	void setPPM(cv::Mat_<double> &P);

	/**
	 * 三次元座標点がこのカメラによってどの位置に投影されるかを計算します.
	 * @return 画素位置
	 * @param [in] p 三次元座標点
	 */
	cv::Point2d project_forw(cv::Point3d p);

	/**
	 * 画像上の点がこのカメラのどの位置からの投影点なのかを計算します.
	 * @return 三次元位置
	 * @param [in] p 画像座標点
	 */
	cv::Point3d project_back(cv::Point2d p);
};




/**
 * 歪みのあるレンズを持つカメラ
 */
class DistortCamera : public PinHoleCamera {
public:
	cv::Mat_<double> D;		///< 歪みパラメータ
    cv::Mat_<double> R2;     ///< カメラ回転行列(平行化前行列保存用)

public:
	/// コンストラクタ
	DistortCamera(cv::Size _size = cv::Size(0, 0), std::string _name = "camdis");

	/// デストラクタ
	virtual ~DistortCamera() {}

	/**
	 * カメラパラメータをファイルストレージに書き出す
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] fs ファイルストレージへのポインタ.0が指定された場合は、create_param_filename()で得られるファイル名で出力する。
	 */
	bool save(cv::FileStorage *_fs = 0);

	/**
	 * カメラパラメータをファイルストレージから読み込む。
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] fs ファイルストレージへのポインタ.0が指定された場合は、create_param_filename()で得られるファイル名で出力する。
	 */
	bool load(cv::FileStorage *_fs = 0);
};




class UndistortCamera : public PinHoleCamera {
public:
	int map_type;			///< 歪み補正マップの要素型(0ならCV_16SC2, 0以外ならCV_32FC1)
	int interpolation_type;	///< 補間タイプ(0:最近傍補間, 1:バイリニア補間, 2:バイキュービック補間, 3:Lanczos法)
	std::string map_name;	///< 歪み補正マップ保存先ファイル名
	cv::Mat Map[2];			///< 歪み補正(0: X成分, 1: Y成分)
	cv::Rect roi;			///< 全てのピクセルが有効である範囲
	cv::Scalar nop;			///< 対応する画素が無い場合に挿入される画素値
    cv::Mat_<double> P2;    ///< 順方向透視投影行列(キャリブ板座標系変換前保存用)


public:
	/// コンストラクタ
	UndistortCamera(cv::Size _size = cv::Size(0, 0), std::string _name = "undiscam")
		: PinHoleCamera(_size, _name), map_type(1), interpolation_type(1), map_name(_name + ".map"), nop(0.0) {}

	/// デストラクタ
	virtual ~UndistortCamera() {}

	/**
	 * このカメラの画像の大きさと名前を設定します.
	 * @param[in] _size 画像の大きさ
	 * @param[in] _name このカメラの名前
	 */
	void set(cv::Size _size, std::string _name) {
		PinHoleCamera::set(_size, _name);
		map_name = _name + ".map";
	}

	/**
	 * カメラパラメータをファイルストレージに書き出す
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] fs ファイルストレージへのポインタ.0が指定された場合は、create_param_filename()で得られるファイル名で出力する。
	 * @param[in] dirname 歪み補正マップ出力先ディレクトリ名
	 */
	bool save(cv::FileStorage *_fs = 0, const char *dirname = 0);

	/**
	 * カメラパラメータをファイルストレージから読み込む。
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] fs ファイルストレージへのポインタ.0が指定された場合は、create_param_filename()で得られるファイル名で出力する。
	 * @param[in] dirname 歪み補正マップ入力先ディレクトリ名
	 */
	bool load(cv::FileStorage *_fs = 0, const char *dirname = 0);

	/**
	 * 歪み補正マップを作成します.
	 * @return なし.
	 * @param[in] camdis 歪み有カメラ
	 */
	void initMap(DistortCamera &camdis);

	/**
	 * 歪み有カメラの画像を歪み無しカメラ画像に変換します.
	 * @return なし.
	 * @param[in]  src 歪み有カメラ画像
	 * @param[out] dst 歪み無しカメラ画像
	 */
	void remap(cv::Mat &src, cv::Mat &dst);
};



/**
 * ステレオカメラクラス
 */
class StereoCamera {
public:
	std::string name;
	UndistortCamera camera[2];
	cv::Mat Q;

public:
	/// コンストラクタ
	StereoCamera(cv::Size _size = cv::Size(0, 0), std::string _name = "stereo");

	/// デストラクタ
	virtual ~StereoCamera() {}

	/**
	 * このクラスのデータを保存するファイル名を返します.
	 * @return ファイル名
	 */
	std::string create_param_filename() { return name + "_param.yaml"; }

	/**
	 * このカメラの画像の大きさと名前を設定します.
	 * @param[in] _size 画像の大きさ
	 * @param[in] _name このカメラの名前
	 */
	void set(cv::Size _size, std::string _name = "stereo");


	/**
	 * 左右カメラのパラメータ、及びQをファイルストレージへ書き出す
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] dirname 出力先ディレクトリ名
	 */
	bool save(const char *dirname = 0);

	/**
	 * カメラパラメータをファイルストレージから読み込む。
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] dirname 入力先ディレクトリ名
	 */
	bool load(const char *dirname = 0);

	/**
	 * 左右カメラのパラメータ(PPM)及びレクティファイテーブルをファイルに出力します.(旧型式用)
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] hmat0_name 左カメラPPM用のファイル名
	 * @param[in] hmat1_name 右カメラPPM用のファイル名
	 * @param[in] rectmap_name レクティファイテーブル用のファイル名
	 */
	bool save_hmats(std::string hmat0_name, const std::string hmat1_name, const std::string rectmap_name);

	/**
	 * 左右カメラのパラメータ(PPM)及びレクティファイテーブルをファイルから読み込みます.(旧型式用)
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] hmat0_name 左カメラPPM用のファイル名
	 * @param[in] hmat1_name 右カメラPPM用のファイル名
	 * @param[in] rectmap_name レクティファイテーブル用のファイル名
	 * @warning これを呼び出す前に必ずカメラの画像サイズを設定しておいてください.
	 */
	bool load_hmats(std::string hmat0_name, const std::string hmat1_name, const std::string rectmap_name);

	/**
	 * Q行列を使用して左右の画素位置から三次元位置を計算します.
	 * @return 三次元位置
	 * @param [in] x 左カメラの画素のX座標値
	 * @param [in] y 左カメラの画素のY座標値
	 * @param [in] d 視差(左カメラのX座標値 - 右カメラのX座標値)
	 */
	cv::Point3d get3dposition_Q(const double x, const double y, const double d);

	/**
	 * 左右の透視投影変換行列から最小二乗法によって三次元位置を計算します.
	 * @return 三次元位置
	 * @param [in] p0 左カメラの画素位置
	 * @param [in] p1 右カメラの画素位置
	 */
	cv::Point3d get3dposition_P(cv::Point2d p0, cv::Point2d p1);

	/**
	 * 左右の画素位置から視線ベクトルを使用して三次元位置を計算します.
	 * @return 三次元位置
	 * @param [in] p0 左カメラの画素位置
	 * @param [in] p1 右カメラの画素位置
	 * @param [out] error 計算誤差を格納する変数への参照
	 */
    cv::Point3d get3dposition(cv::Point2d p0, cv::Point2d p1, double &error);
    int genPC(Eigen::MatrixXd &diff, Eigen::MatrixXp &tex, Eigen::MatrixXp &mask, Eigen::MatrixXpt &output,double th_err =3.0e-12);
};
