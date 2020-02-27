#pragma once

#include "camera_pinhole.hpp"
#include "camera_distort.hpp"
#include <fstream>

/// レクティファイマップデータのアライメント
#define MAP_DATA_ALIGNMENT	(8)


/**
 * 歪み無しカメラ(歪み有りカメラの入力画像を補正して、歪み無しで撮影した画像に補正する)
 * 歪み補正テーブルとROIを保持する
 */ 
class UndistortCamera : public PinHoleCamera {
public:
	int map_type;			///< 歪み補正マップの要素型(0ならCV_16UC1, 0以外ならCV_32FC1)
	int interpolation_type;	///< 補間タイプ(0:最近傍補間, 1:バイリニア補間, 2:バイキュービック補間, 3:Lanczos法)
	cv::Mat Map[2];			///< 歪み補正(0: X成分, 1: Y成分)
	cv::Rect roi;			///< 全てのピクセルが有効である範囲
	cv::Scalar nop;			///< 対応する画素が無い場合に挿入される画素値


public:
	/// コンストラクタ
	UndistortCamera(cv::Size _size = cv::Size(0, 0), std::string _name = "undiscam")
		: PinHoleCamera(_size, _name), map_type(1), interpolation_type(1), nop(0.0) {}

	/// デストラクタ
	virtual ~UndistortCamera() {}

	/// コピーコンストラクタ
	UndistortCamera(const UndistortCamera &obj);

	/// 代入演算子
	UndistortCamera& operator=(const UndistortCamera &obj);

	/**
	 * カメラパラメータをファイルストレージに書き出す
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] fs ファイルストレージへのポインタ.0が指定された場合は、create_param_filename()で得られる
	 * ファイル名で出力する。
	 */
	bool save(cv::FileStorage *_fs = 0, const std::string rmap_filename = std::string());

	/**
	 * カメラパラメータをファイルストレージから読み込む。
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] fs ファイルストレージへのポインタ.0が指定された場合は、create_param_filename()で得られる
	 * ファイル名で出力する。
	 */
	bool load(cv::FileStorage *_fs = 0, const std::string rmap_filename = std::string());

	/**
	 * 歪み補正マップを作成します.
	 * @return なし.
	 * @param[in] oldcam レクティファイ前カメラ
	 * @param[in] oldcam_rotate レクティファイによるカメラの回転(行列)
	 */
	void initMap(DistortCamera &oldcam, cv::Mat_<double> &oldcam_rotate);

	/**
	 * 歪み有カメラの画像を歪み無しカメラ画像に変換します.
	 * @return なし.
	 * @param[in]  src 歪み有カメラ画像
	 * @param[out] dst 歪み無しカメラ画像
	 */
	void remap(cv::Mat &src, cv::Mat &dst);
};


/**
 * レクティファイマップ(旧方式)でファイルに出力します.
 * @return なし.
 * @param [in|out] undcam 歪み無しカメラ
 * @param [in] ifs レクティファイマップを保存するファイルの出力ストリーム
 */
void SaveRectifyTables(UndistortCamera undcam[2], std::ofstream &ofs);


/**
 * レクティファイマップ(旧方式)をファイルから読み取ります.
 * @return 読み取りに成功した場合はtrue, 失敗した場合はfalse.
 * @param [in|out] undcam 歪み無しカメラ(UndistortCamera::Mapにレクティファイマップを保存します)
 * @param [in] ofs レクティファイマップが保存されているファイルの入力ストリーム
 * @warning UndistortCameraには画像サイズを設定しておく必要があります。この画像サイズは
 * レクティファイマップのサイズと同じでなければなりません。
 */
bool LoadRectifyTables(UndistortCamera undcam[2], std::ifstream &ifs);
