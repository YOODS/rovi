#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <fstream>

#define CAMERAPARAM_FORMAT_STRING    ("yaml")


inline cv::Point2d project3d_2d(cv::Mat_<double> &Pmat, cv::Point3d &p)
{
	cv::Mat_<double> X = (cv::Mat_<double>(4, 1) << p.x, p.y, p.z, 1.0);
	cv::Mat_<double> Y = Pmat * X;
	double s = 1.0 / Y[2][0];
	return cv::Point2d(s * Y[0][0], s * Y[1][0]);	
}



/**
 * ピンホールカメラモデル
 */
class PinHoleCamera {
public:
	cv::Size size;			///< 画像サイズ		
	std::string name;		///< カメラ名

	cv::Mat_<double> K;		///< 内部パラメータ行列
	cv::Mat_<double> R;		///< カメラ回転行列
	cv::Mat_<double> T;		///< カメラ並進ベクトル
	cv::Mat_<double> fP;	///< 順方向透視投影行列
	cv::Mat_<double> rP;	///< 逆方向透視投影行列
	cv::Mat_<double> RT;    ///< 回転行列 * カメラ並進ベクトル
	cv::Mat_<double> iRT;   ///< RTの逆行列


public:
	/**
	 * コンストラクタ
	 * @param[in] _name このカメラの名前
	 * @param[in] _size 画像の大きさ
	 */
	PinHoleCamera(cv::Size _size = cv::Size(0, 0), std::string _name = "camph");

	/// デストラクタ
	virtual ~PinHoleCamera() {}

	/// コピーコンストラクタ
	PinHoleCamera(const PinHoleCamera &obj);

	/// 代入演算子
	PinHoleCamera &operator=(const PinHoleCamera &obj);

	/**
	 * 全てのカメラパラメータを保存するファイル名を返します.
	 * @return ファイル名
	 */
	std::string create_param_filename(void) { return name + "_param." + CAMERAPARAM_FORMAT_STRING; }
	
	/**
	 * カメラパラメータをファイルストレージに書き出す
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] fs ファイルストレージへのポインタ.0が指定された場合は、create_param_filename()
	 * で得られるファイル名で出力する。
	 */
	virtual bool save(cv::FileStorage *_fs = 0);

	/**
	 * カメラパラメータをファイルストレージから読み込む。
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] fs ファイルストレージへのポインタ.0が指定された場合は、create_param_filename()
	 * で得られるファイル名で出力する。
	 */
	virtual bool load(cv::FileStorage *_fs = 0);

	
	/**
	 * PPMを指定されたファイルストリームに書き出す.(旧型式用)
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] ofs 出力用ファイルストリーム
	 */
	bool save_hmat(std::ofstream &ofs);


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
	 * 画像上の点がどの三次元位置からの投影点なのかを計算します.
	 * @return 三次元位置
	 * @param [in] p 画像座標点
	 */
	cv::Point3d project_back(cv::Point2d p);

	/**
	 * 光学中心から二次元画像点へ向かう視線ベクトルを計算します.
	 * @return 視線ベクトル
	 * @param [in] p 画像座標点
	 */	
	cv::Vec3d get_sight_vector(cv::Point2d p);

	/**
	 * カメラの位置を返します.(座標系は透視投影行列と同じ)
	 * @return カメラ位置
	 */	
	cv::Mat_<double> get_center() const;

	virtual bool solvePnPRansac(cv::Mat &rvec, cv::Mat &tvec, std::vector<cv::Point2f> &point2d, std::vector<cv::Point3f> &point3d);
	virtual bool solvePnP(cv::Mat &rvec, cv::Mat &tvec, std::vector<cv::Point2f> &point2d, std::vector<cv::Point3f> &point3d);

};
