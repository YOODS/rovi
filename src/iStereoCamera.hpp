#pragma once
#include <vector>
#include <string>


class iStereoCamera {
public:
	enum { LCAM = 0, RCAM = 1, N_CAMERA = 2 };

	virtual void destroy() = 0;

	/**
	 * 基線長を返します
	 * @return 基線長
	 */
	virtual const double get_baseline_length(void) const = 0;

	/**
	 * ステレオカメラの入力画像のサイズを取得します.
	 * @return なし
	 * @param [out] wd 入力画像の横幅を格納する変数へのポインタ
	 * @param [out] ht 入力画像の縦幅を格納する変数へのポインタ 
	 * @note remapで与える入力画像の大きさは必ずここで得られたサイズとしてください.
	 */
	virtual void get_inpimage_size(int *wd, int *ht) const = 0;

	/**
	 * ステレオカメラの出力画像のサイズを取得します.
	 * @return なし
	 * @param [out] wd 出力画像の横幅を格納する変数へのポインタ
	 * @param [out] ht 出力画像の縦幅を格納する変数へのポインタ
	 * @note remapで与える出力画像の大きさは必ずここで得られたサイズとしてください.
	 */
	virtual void get_outimage_size(int *wd, int *ht) const = 0;

	/**
	 * 与えられた入力画像をステレオカメラの画像に変換(レクティファイ)します.
	 * @return 変換に成功した場合はtrue, 失敗した場合はfalse.
	 * @param [out] o_image 出力画像左上端点のアドレス
	 * @param [in] o_step 出力画像バッファの水平方向のバイト数
	 * @param [in] i_image 入力画像左上端点のアドレス
	 * @param [in] i_step 入力画像バッファの水平方向のバイト数
	 * @param [in] cam 左カメラの場合は0, 右カメラの場合は1を与える.入力画像が左右連結である場合は2を与える。
	 * @note cam==2とする場合は必ずo_imageは出力画像サイズ×2のメモリを確保してください.
	 */
	virtual bool remap(unsigned char *o_image, const size_t o_step,
		unsigned char *i_image, const size_t i_step, const int cam) = 0;

	/**
	 * 視差から三次元位置を計算します.
	 * @return なし.
	 * @param [out] p 計算された三次元位置を格納するバッファへのポインタ(要素は3個. x, y, z用)
	 * @param [in] x, y 左カメラの画素の座標値
	 * @param [in] disparity 視差(左カメラのX座標値 - 右カメラのX座標値)
	 * @note x, y, disparityはレクティファイ画像で得られた値を与えてください.
	 */
	virtual bool get3dposition(double *p, const int x, const int y, const double disparity) = 0;

	/**
	 * 左右の透視投影変換行列(P行列)から最小二乗法によって三次元位置を計算します.
	 * @return なし.
	 * @param [out] p 計算された三次元位置を格納するバッファへのポインタ(要素は3個. x, y, z用)
	 * @param [in] pL 左カメラの画素位置を格納するバッファへのポインタ(要素は2個. i, j用)
	 * @param [in] pR 右カメラの画素位置を格納するバッファへのポインタ(要素は2個. i, j用)
	 * @note pL, pR はレクティファイ画像で得られた値を与えてください.
	 */
	virtual bool get3dposition(double *p, const double *pL, const double *pR) = 0;

	/**
	 * 左右の画素位置から視線ベクトルを使用して三次元位置を計算します.
	 * @return 三次元位置
	 * @param [out] p 計算された三次元位置を格納するバッファへのポインタ(要素は3個. x, y, z用)
	 * @param [in] ethr 計算誤差の上限値
	 * @param [in] p0 左カメラの画素位置を格納するバッファへのポインタ(要素は2個. i, j用)
	 * @param [in] p1 右カメラの画素位置を格納するバッファへのポインタ(要素は2個. i, j用)
	 * @note pL, pR はレクティファイ画像で得られた値を与えてください.
	 */
	virtual bool get3dposition(double *p, const double ethr, const double *pL, const double *pR) = 0;
};


/// ステレオカメラ作成のための設定
struct StereoCameraSettings {
	int input_cols;		///< 入力(カメラ)画像の横幅
	int input_rows;		///< 入力(カメラ)画像の縦幅

	int output_cols;	///< 出力画像の横幅
	int output_rows;	///< 出力画像の縦幅

	double alpha;		///< レクティファイのスケーリング値
	int flags;			///< レクティファイのフラグ値

	unsigned char nopL;	///< レクティファイ後に左カメラ画像の余白に挿入される輝度値
	unsigned char nopR;	///< レクティファイ後に右カメラ画像の余白に挿入される輝度値

	StereoCameraSettings() : alpha(-1.0), flags(0), nopL(0), nopR(0) {}
};



#ifdef _WINDOWS

#ifdef _WINDLL
#define EXPORT_STEREO_CAMERA	__declspec(dllexport)
#else
#define EXPORT_STEREO_CAMERA	__declspec(dllimport)
#endif	// _WINDLL

#else	// else _WINDOWS

#define EXPORT_STEREO_CAMERA

#endif	// _WINDOWS


enum CamParamType {
	HMat = 1,	///< HMat形式のステレオカメラファイルsからステレオカメラを作成する
	CamN = 2,	///< cam0_param.yaml, cam1_param.yaml形式のカメラパラメータファイルからステレオカメラを作成する
};

/**
  ステレオカメラを作成します.
  @return 作成されたステレオカメラのインスタンス
  @param [in] camtype CameraTypeの何れかの値
  @param [in] settings ステレオカメラ作成のための設定

  @note CamParamTypeによって、必要なファイルの数と名前が異なるので注意.
  - CamParamType::HMat の場合. filenames[0] = 左カメラ用Hmat, filenames[1] = 右カメラ用Hmat, filenames[2] = rect.param
  - CamParamType::CamN の場合. filenames[0] = 左カメラのカメラパラメータファイル, filenames[1] = 右カメラのカメラパラメータファイル
 */
extern "C" EXPORT_STEREO_CAMERA iStereoCamera* CreateStereoCamera(const CamParamType camtype,
	const std::vector<std::string> filenames, StereoCameraSettings* settings);

typedef iStereoCamera *(*pCreateStereoCamera)(const int camtype,
	const std::vector<std::string> filenames, StereoCameraSettings* settings);



/**
 * カメラパラメータデータを内部にコピーします.
 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
 * @param [in] Kl, Kr 左右のカメラパラメータ(3x3=9要素が格納されているvector<double>)
 * @param [in] Dl, Dr 左右の歪みパラメータ(0 or 5 or 14 要素が格納されているvector<double>)
 * @param [in] R 右カメラ->左カメラの回転行列(3x3=9要素が格納されているvector<double>)
 * @param [in] T 右カメラ->左カメラの移動ベクトル(3x1=3要素が格納されているvector<double>)
 * @param [in] settings ステレオカメラ作成のための設定
 */
extern "C" EXPORT_STEREO_CAMERA iStereoCamera* CreateStereoCameraFromRaw(
	std::vector<double> &Kl, std::vector<double> &Kr,
	std::vector<double> &Dl, std::vector<double> &Dr,
	std::vector<double> &R, std::vector<double> &T, StereoCameraSettings* settings);

typedef iStereoCamera *(*pCreateStereoCameraFromRaw)(
	std::vector<double> &Kl, std::vector<double> &Kr,
	std::vector<double> &Dl, std::vector<double> &Dr,
	std::vector<double> &R, std::vector<double> &T, StereoCameraSettings* settings);
