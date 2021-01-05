#pragma once

#ifdef _WINDOWS
#ifdef _WINDLL
#define EXPORT_PCGEN __declspec(dllexport)
#else
#define EXPORT_PCGEN __declspec(dllimport)
#endif
#else
#define EXPORT_PCGEN
#endif

#include <string>
#include <limits>
#include "iStereoCamera.hpp"


class PointCloudCallback {
public:
	struct Point3d {
		float x, y, z;

		// コンストラクタ
		// @param _x, _y, _z 三次元座標値
		// @note デフォルト値は無効な点座標値を表す値std::numeric_limits<float>::quiet_NaN()である.
		Point3d(const float _x = std::numeric_limits<float>::quiet_NaN(), 
				const float _y = std::numeric_limits<float>::quiet_NaN(),
				const float _z = std::numeric_limits<float>::quiet_NaN()) : x(_x), y(_y), z(_z) {}
	};

	/**
	  iPointCloudGenerator.get_pointcloud(this)と渡したときに呼び出されるコールバック関数.
	  @return 無し
	  @param [in] image 点群に張り付けられるテクスチャ画像の左上端アドレス
	  @param [in] step テクスチャ画像バッファの水平方向のバイト数
	  @param [in] width テクスチャ画像の横幅
	  @param [in] height テクスチャ画像の縦幅
	  @param [in] points 点群の三次元座標値(テクスチャ画像をZスキャンした順に並んでいます)
	  @param [in] n_valid 有効な点の数
	  @note image(i, j)に対応する三次元点はpoints[i + j * width]に格納されています.その点が有効な点か否かは座標値に
	  std::numeric_limits<float>::quiet_NaN()が格納されているか否かで判断してください.
	 */
	virtual void operator()(
		unsigned char *image, const size_t step,
		const int width, const int height, 
		std::vector<Point3d> &points, const int n_valid) = 0;
};



class iPointCloudGenerator {
public:
	/**
	 * 点群生成器を破棄します.
	 * @return なし
	 */
	virtual void destroy() = 0;

	/**
	 * 点群生成器の初期化を行います.
	 * @return なし.
	 * @param [in] cam 使用するステレオカメラのインスタンス
	 */
	virtual void init(iStereoCamera *cam) = 0;

	/**
	 * 三次元点の座標変換行列をセットします.
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] filename 4x4の剛体変換行列が格納されているファイル名(OpenCVのMat形式)
	 */
	virtual bool convert_coordinate(const std::string filename) = 0;

	/**
	 * 三次元点の座標変換行列をセットします.
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] RT 4x4の剛体変換行列が格納されているvector(4x4=16要素)
	 */
	virtual bool convert_coordinate(std::vector<double> &RT) = 0;

	/**
	 * 点群生成に必要なパラメータをセットします.
	 * @param [in] params パラメータ型へのポインタ
	 * @note params は手法によって異なります.グレイコード版位相シフトの場合はParamPSFT.hppを、
	 * 三位相版位相シフトの場合はParamMPSFT.hppを、SGBMの場合はParamSGBM.hppを参照してください.
	 */
	virtual bool setparams(void *params) = 0;

	/**
	 * 点群を生成するために必要な画像枚数
	 * @return 必要な画像枚数
	 * @note 1はSGBMの場合. 位相シフトの場合は手法によって異なる
	 */
	virtual const size_t requiredframes() const { return 1; }


	/**
	 * 点群生成器をリセットします.(内部に残っているプロジェクタ画像を削除します.SGBMでは何もしません.)
	 */
	virtual bool reset() { return true; }

	/**
	 * 処理対象画像を点群生成器に渡します.
	 * @return 成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] grayim 画像バッファ左上端アドレス(白黒画像のみ可)
	 * @param [in] step 画像バッファの水平方向のバイト数
	 * @param [in] cam カメラ番号
	 * @param [in] idx 画像番号(位相シフトの場合必ず必要)
	 * @note 左右連結画像(画像左側が左カメラ画像、右側が右カメラ画像となるように連結された画像)を与える場合には、stepは連結画像の
	 * 水平方向のバイト数を与え、カメラ番号は2としてください.
	 */
	virtual bool setpict(void *grayim, const size_t step, const int cam, const int idx = 0) = 0;

	/**
	 * 指定されたファイルから処理対象画像を読み込んで、点群生成器に渡します.
	 * @return 読み込み(＆点群生成器にセット)が成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] filename 画像ファイル名
	 * @param [in] cam セットするカメラ番号(0: 左カメラ, 1: 右カメラ, 2: 両カメラ(但しこれは連結画像のみ正常に動作する)
	 * @param [in] idx 画像番号(位相シフトの場合必ず必要)
	 * @note 左右連結画像(画像左側が左カメラ画像、右側が右カメラ画像となるように連結された画像)を与える場合には、カメラ番号は2
	 * としてください.
	 */
	virtual bool loadpict(std::string filename, const int cam, const int idx = 0) = 0;

	/**
	 * setpict or loadpictによって点群生成器に渡された画像を、レクティファイして返します.
	 * @return 成功した場合はtrue, 失敗した場合はfalse.
	 * @param top [out] 画像先頭アドレスを格納する変数へのポインタ(外で解放しないでください)
	 * @param width [out] 画像横幅を格納する変数へのポインタ
	 * @param height [out] 画像縦幅を格納する変数へのポインタ
	 * @param step [out] 画像バッファの水平方向のバイト数
	 * @param [in] cam カメラ番号(0: 左カメラ, 1: 右カメラ, 2: 連結画像)
	 * @param [in] idx 画像番号(位相シフトの場合のみ必要.SGBMでは無視されます). 0: 黒画像, 1: 白画像となる.
	 * @warning setpict() or loadpict()呼び出し後、preprocess()呼び出し前に呼び出してください. 
	 */
	virtual bool getpict(unsigned char **top, int *width, int *height, size_t *step, 
		const int cam, const int idx = 0) = 0;

	/**
	 * 点群生成器に蓄えられた画像sからプロジェクタ座標画像を作成します.(SGBMでは何もしません)
	 * @return 処理が成功した場合はtrue, 失敗した場合はfalse.
	 * @warning this->requiredframes()から返される枚数分setpict() or loadpict()を呼び出した後に必ず
	 * この関数を呼び出してください.
	 */
	virtual bool flushImageBuffer() { return true; }


	/**
	 * 点群生成のための前準備を行います.
	 * @return なし.
	 */
	virtual bool preprocess() = 0;

	/**
	 * 視差マップを作成します.
	 * @return なし.
	 * @param [in] texture_cam 点群に張り付けるテクスチャをどちらのカメラからのものを使用するか?
	 * (0: 左カメラ(従来通り), 1: 右カメラ)
	 */
	virtual void make_disparitymap(const int texture_cam_ = 0) = 0;

	/// 三次元座標値を計算する手法
	enum Method3D {
		QMatrix = 0,	///< Q行列
		SVector,		///< 視線ベクトル方式
		PMatrix			///< P行列
	};

	/**
	 * 点群データを作成し、内部バッファに保存します.
	 * @param [in] method 三次元座標値を計算する手法(Method3D参照)
	 */
	virtual void generate_pointcloud(const Method3D method = Method3D::QMatrix) = 0;


	/**
	 * 作成された点群データを取得します.(OnRevcPointCloud.operator()を呼び出します)
	 * @return 有効な点の数
	 * @param [in] callback コールバック関数クラスインスタンスへのポインタ
	 */
	virtual int get_pointcloud(PointCloudCallback* callback) = 0;
};



/**
 * 点群生成モード
 */
enum PcGenMode {
	PCGEN_SGBM = 0,	///< SGBM
	PCGEN_GRAYPS4,	///< 位相シフト: Gray + 4step PS
	PCGEN_MULTI,	///< 位相シフト: マルチ
};

/**
 * 点群生成器を作成します.
 * @return 点群生成器インスタンスへのポインタ
 * @param [in] mode 点群生成手法(PcGenModeの何れか)
 */
extern "C" EXPORT_PCGEN iPointCloudGenerator* CreatePointCloudGenerator(const int mode = 0);
typedef iPointCloudGenerator *(*pCreatePointCloudGenerator)(const int mode);
