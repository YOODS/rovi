#pragma once
#include "camera_distort.hpp"
#include "camera_undistort.hpp"

/**
 * ステレオカメラクラス
 */
class StereoCamera {
public:
	enum { LCAM = 0, RCAM = 1, N_CAMERA = 2 };
	UndistortCamera camera[2];
	cv::Mat Q;
	std::string name;
	cv::Size oldcam_size;
	cv::Mat_<double> oldcamR[2];	///< レクティファイによる旧カメラの回転
	cv::Mat_<double> baseline;		///< 基線ベクトル


public:
	/**
	 * コンストラクタ. 初期化のみ
	 */
	StereoCamera(cv::Size _size = cv::Size(0, 0));
	   
	/**
	 * デストラクタ
	 */
	virtual ~StereoCamera() {}
	
	/**
	 * コピーコンストラクタ
	 */
	StereoCamera(const StereoCamera &obj);
	
	/**
	 * 代入演算子
	 */
	StereoCamera& operator=(const StereoCamera &obj);
			
	/**
	 * コンストラクタ. 歪み有りカメラのカメラパラメータからステレオカメラを作成します. 
	 * @param discam [in] 歪み有りカメラ(0: 左カメラ, 1: 右カメラ)
	 * @param alpha [in] スケーリングパラメータ. (cv::stereoRectifyのalphaと同じ)
	 * @param flags [in] 処理フラグ(cv::stereoRectifyのflagsと同じ)
	 * @param newImageSize [in] 平行化後の新しい画像の解像度.(0, 0)が与えられると元画像サイズに設定される
	 * @param nopL [in] 平行化後に対応する元画像の画素が無い場合に挿入される画素値(左カメラ用)
	 * @param nopR [in] 平行化後に対応する元画像の画素が無い場合に挿入される画素値(右カメラ用)
	 *
	 * @warning 歪み有りカメラは、0番目のカメラを基準とした1番目のカメラのRTが、1番目カメラの
	 * パラメータファイルに記載してあることを前提としています. 
	 */	
	StereoCamera(DistortCamera discam[2], const double alpha = -1.0, const int flags = 0,
				 cv::Size newImageSize = cv::Size(0, 0),
				 cv::Scalar nopL = cv::Scalar(0, 0, 0), cv::Scalar nopR = cv::Scalar(0, 0, 0));

	/**
	 * このステレオカメラが水平ステレオであるか判定します.
	 * @return 水平ステレオカメラの場合true, そうでない(垂直ステレオカメラの)場合はfalse
	 */
	bool is_horizontal_stereo();


	/**
	* デフォルトのパラメータファイル名を返します.
	* @return パラメータファイル名
	*/
	std::string create_param_filename() const { return name + "_param." + CAMERAPARAM_FORMAT_STRING;  }

	/**
	 * 基線長の長さを返します.
	 * @return 基線長
	 */
	double get_baseline_length() {
		return cv::norm(this->baseline);
	}

	/**
	 * 与えられた画像サイズがステレオカメラを生成した時の入力画像サイズと同じか否かを判定する.
	 * @return 入力画像サイズが同じであればtrue, 異なっていればfalse.
	 * @param size 画像サイズ
	 *
	 * @warning 入力画像サイズがキャリブレーション時のサイズと異なっている場合、レクティファイを
	 * 行うと、cv::remapが例外を投げます.
	 */
	bool check_input_image_size(cv::Size &size) const {
		return size == this->oldcam_size;
	}

	/**
	 * 与えられた歪み有りカメラでの撮影画像をレクティファイして、ステレオカメラ画像に変換します.
	 * @return レクティファイされた画像
	 * @param source 左右連結カメラ画像
	 */
	cv::Mat remap(cv::Mat &source);

	
	/**
	 * Q行列を現在のカメラパラメータから再計算します.(Hmatからパラメータを読み込んだときに使用すると
	 * Q行列を作成できる.)
	 * @return なし.
	 */
	void recalculationQ();
	
	/**
	 * 左右カメラのパラメータ、及びQをファイルストレージへ書き出す
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] directory 出力先ディレクトリ名
	 */
	bool save(const std::string directory = std::string("./"));

	/**
	 * カメラパラメータをファイルストレージから読み込む。
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] directory 入力先ディレクトリ名
	 */
	bool load(const std::string directory = std::string("./"));

	
	/**
	 * カメラ中央から与えられた距離だけ離れた点における視差を計算します.
	 * @return 視差
	 * @param distance カメラ中央からの距離(単位はキャリブレーション板に準じる)
	 */
	double convert_distance_to_disparity(const double distance);
	
	/**
	 * Q行列を使用して左右の画素位置から三次元位置を計算します.
	 * @return 三次元位置
	 * @param [in] x 左カメラの画素のX座標値
	 * @param [in] y 左カメラの画素のY座標値
	 * @param [in] d 視差(左カメラのX座標値 - 右カメラのX座標値)
	 */
	cv::Point3d get3dposition_Q(const double x, const double y, const double d);

	/**
	 * 左右の透視投影変換行列(P行列)から最小二乗法によって三次元位置を計算します.
	 * @return 三次元位置
	 * @param [in] p0 左カメラの画素位置
	 * @param [in] p1 右カメラの画素位置
	 * @param [out] status 計算が成功したか否かの状態値を格納する変数へのポインタ
	 */
	cv::Point3d get3dposition_P(cv::Point2d p0, cv::Point2d p1, bool *status = 0);

	/**
	 * 左右の画素位置から視線ベクトルを使用して三次元位置を計算します.
	 * @return 三次元位置
	 * @param [in] p0 左カメラの画素位置
	 * @param [in] p1 右カメラの画素位置
	 * @param [out] error 計算誤差を格納する変数への参照
	 */
	cv::Point3d get3dposition(cv::Point2d p0, cv::Point2d p1, double &error);


	cv::Point3d position_from_sv(cv::Vec3d svL, cv::Vec3d svR, double &error,
		cv::Vec3d &cam0_center, cv::Vec3d &cam1_center);

	/**
	 * 与えられた左右画像上での座標値と対応する三次元位置から物体の姿勢を求めます.
	 * @return 処理が成功した場合はtrue, 失敗した場合はfalse.
	 * @param [out] RT 物体の姿勢(4x4行列)
	 * @param [in] point2d 左右画像上でのマーカ位置(但し、未検出マーカは取り除かれている)
	 * @param [in] point3d 上記マーカの三次元位置
	 */
	bool solvePose(cv::Mat_<double> &RT, std::vector< std::vector<cv::Point2f> > &point2d, std::vector<cv::Point3f> &point3d);

	/**
	 * このステレオカメラの座標系を変換する.
	 * @return なし
	 * @param [in] RT 希望する座標系のRT(希望する座標系->カメラ座標系の変換行列. 4x4)
	 */
	void convert_coordinate_system(cv::Mat_<double> &RT);
};


/**
 * 二台のカメラパラメータからステレオカメラを生成します.
 *
 * @return 生成されたステレオカメラインスタンスのアドレス
 * @param paramfiles[in] カメラパラメータファイル名s(左右で異なる場合は0番目に左、1番目に右のファイル名を指定してください)
 * @param alpha [in] スケーリングパラメータ. (cv::stereoRectifyのalphaと同じ)
 * @param flags [in] 処理フラグ(cv::stereoRectifyのflagsと同じ)
 * @param newImageSize [in] 平行化後の新しい画像の解像度.(0, 0)が与えられると元画像サイズに設定される
 * @param nopL [in] 平行化後に対応する元画像の画素が無い場合に挿入される画素値(左カメラ用)
 * @param nopR [in] 平行化後に対応する元画像の画素が無い場合に挿入される画素値(右カメラ用)
 *
 * @warning 二台のカメラパラメータが一つのファイルに格納されている場合は、paramfilesにはそのファイル名のみ
 * を格納するか、同じファイル名を二つ格納してください.
 * また、生成されたステレオカメラはdeleteによって破棄してください.
 */
StereoCamera* create_StereoCamera(std::vector<std::string> paramfiles,
								  const cv::Size newImageSize = cv::Size(0, 0),								  
								  const double alpha = -1.0, const int flags = 0,
								  cv::Scalar nopL = cv::Scalar(0, 0, 0), cv::Scalar nopR = cv::Scalar(0, 0, 0));


/**
 * 左右カメラのパラメータ(PPM)及びレクティファイテーブルをファイルから読み込み、ステレオカメラを
 * 生成します.(旧型式用)
 *
 * @return 処理に成功した場合はStereoCameraのインスタンスへのポインタ, 失敗した場合はNull.
 * @param[in] hmat0_name 左カメラPPM用のファイル名
 * @param[in] hmat1_name 右カメラPPM用のファイル名
 * @param[in] rectmap_name レクティファイテーブル用のファイル名
 * @warning これを呼び出す前に必ずカメラの画像サイズを設定しておいてください.
 */
StereoCamera* create_StereoCamera_from_Hmats(
	std::string hmat0_name, const std::string hmat1_name, const std::string rectmap_name,
	cv::Size imageSize, cv::Scalar nopL = cv::Scalar(0, 0, 0), cv::Scalar nopR = cv::Scalar(0, 0, 0));



/**
 * 左右カメラのパラメータ(PPM)及びレクティファイテーブルをファイルに出力します.(旧型式用)
 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
 * @param[in] stereo ステレオカメラのアドレス
 * @param[in] hmat0_name 左カメラPPM用のファイル名
 * @param[in] hmat1_name 右カメラPPM用のファイル名
 * @param[in] rectmap_name レクティファイテーブル用のファイル名
 */
bool store_StereoCamera_to_Hmats(StereoCamera *stereo,
								 std::string hmat0_name, const std::string hmat1_name,
								 const std::string rectmap_name);

