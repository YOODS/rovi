#pragma once

#include "iCalibBoardRecognizer.hpp"
#include <string>


/**
 * キャリブレーションボード姿勢推定器インターフェース
 */
class iCalibBoardPoseSolver {
public:
	/**
	 * デストラクタ呼び出し
	 */
	virtual void destroy() = 0;

	/**
	 * カメラの初期化を行います.
	 * @return 初期化に成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] filenames ファイル名や文字列が格納されたリスト.必要な項目はcamera_typeによって異なる。
	 * @param [in] camera_type カメラタイプ.以下の通り.
	 * - 0: 歪み有カメラ. filenames[0]には歪み有カメラのカメラパラメータファイル名を格納すること.
	 *
	 * - 1: ステレオ左カメラ. 二つの歪み有カメラからステレオ平行化カメラを作成し、その左カメラを姿勢推定
	 *		に使用する.filenames[0]には左側歪み有カメラのカメラパラメータファイル(cam0_param.yaml等)を、
	 *		filenames[1]には右側歪み有カメラのカメラパラメータファイル(cam1_param.yaml等)を指定すること.
	 *
	 * - 2: StereoParam歪み補正カメラ. filenames[0]にはステレオパラメータファイル名、filenames[1]には
	 *		左カメラの平行化マップファイル名を格納すること(右はあっても使わないので要らない).
	 *
	 * - 3: Hmat形式ステレオ歪み補正カメラ. filenames[0]には左カメラのHmat、filenames[1]には右カメラのHmat、
	 *		filenames[2]には平行化マップのファイル名を格納すること. また、このカメラの場合、入力画像のサイズが
	 *		必要になるので必ず, image_width, image_heightを指定すること.
	 *
	 * - 4: ピンホールカメラ. filenames[0]にはピンホールカメラのカメラパラメータファイルを指定すること.この
	 *		場合は入力画像は、ピンホールカメラで撮影した様に変換されていなければならない.
	 */
	virtual bool init_camera(const std::vector<std::string> &filenames, const int camera_type = 0,
		const int image_width = 0, const int image_height = 0) = 0;

	/**
	 * キャリブレーションボードのパラメータ設定を行います.
	 * @return パラメータ推定に成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] pp マーカ検出のための前処理用パラメータ
	 * @param [in] mp 円マーカ検出のためのパラメータ
	 * @param [in] cp キャリブボードのパラメータ
	 */
	virtual bool set_parameters(PreProcParam &pp, CircleMarkerParam &mp, CalibBoardParam &cp) = 0;


	// ボードの姿勢推定 ------------------------------------------------------

	/**
	 * 与えられたキャリブボード画像からボードの姿勢を推定します.
	 * @return 推定された姿勢を使って得られたマーカ点の再投影誤差(平均値). 姿勢推定に失敗したら負の値
	 * @param[in] image  入力画像左上端アドレス
	 * @param[in] width  画像横幅
	 * @param[in] height 画像縦幅
	 * @param[in] step   画像バッファの水平方向のバイト数
	 * @param [out] pose 推定されたボードの姿勢(前3要素移動ベクトル、後３要素回転ベクトル)
	 */
	virtual double solve_pose(unsigned char *image, const int width, const int height, const size_t step,
		double *pose = 0) = 0;

	/**
	 * 与えられたファイル名から画像を読み込み、ボードの姿勢を推定します.
	 * @return 推定された姿勢を使って得られたマーカ点の再投影誤差(平均値). 姿勢推定に失敗したら負の値
	 * @param [in] imagename 画像ファイル名
	 * @param [in] is_connected 連結画像の場合はtrue, そうでなければfalse.(連結画像の場合は左側のみ使用します)
	 * @param [out] pose 推定されたボードの姿勢(前3要素移動ベクトル、後３要素回転ベクトル)
	 */
	virtual double solve_pose(const std::string filename, const bool is_connected = false, double *pose = 0) = 0;

	// 出力 ----------------------------------------------------------------

	/**
	 * 検出されたマーカを描画した画像を指定されたファイル名で保存します.
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param filename 画像ファイル名
	 */
	virtual bool save_debug_image(const std::string filename) = 0;

	/**
	 * 画面にマーカ検出結果を表示します.
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 */
	virtual bool show_debug_image() = 0;

	/**
	 * マーカ検出結果画像(RGBカラー)バッファの先頭アドレスを返します.
	 * @return バッファ先頭アドレス
	 */
	virtual unsigned char* ptr_debug_image() = 0;

	/**
	 * マーカ検出結果画像のサイズを求めます.
	 * @return なし
	 * @param [out] width 結果画像の横幅
	 * @param [out] height 結果画像の縦幅
	 * @warning  レクティファイが実行された場合、入力画像とマーカ検出対象画像のサイズが異なることがあります.
	 */
	virtual void get_size(int *width, int *height) = 0;
};


#ifdef _WINDOWS
#ifdef _WINDLL
#define EXPORT_CBSOLVER __declspec(dllexport)
#else
#define EXPORT_CBSOLVER __declspec(dllimport)
#endif
#else
#define EXPORT_CBSOLVER
#endif

/**
 * ハンドアイキャリブレーションクラスインスタンス作成関数
 * @return クラスインスタンスのアドレス
 */
extern "C" EXPORT_CBSOLVER iCalibBoardPoseSolver* CreateCalibBoardPoseSolver();

typedef iCalibBoardPoseSolver *(*pCreateCalibBoardPoseSolver)();

