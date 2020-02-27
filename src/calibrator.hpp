#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "iCalibBoardRecognizer.hpp"
#include "camera_distort.hpp"


/// カメラキャリブパラメータ型
struct CameraCalibParams {
	double leng;	///< 焦点距離
	double cell;	///< センサーセルサイズ
	double F;				///< カメラパラメータF値
	int calibration_flags;	///< cv::cameraCalibrateのフラグ値

	CameraCalibParams() :
		leng(0.0), cell(0.0), F(0.0), calibration_flags(0) {}

	void setF() {
		if (leng == 0.0 || cell == 0.0) F = 0.0;
		else {
			F = leng / cell;
		}
	}
};


/**
 * キャリブレーションクラス
 */
class CameraCalib {
public:
	/**
	 * コンストラクタ.
	 * @param [in] _type キャリブレーションボードタイプ
	 */
	CameraCalib(const int _type = 0) : recognizer(0), calib_param(), board_type(_type) {}
	virtual ~CameraCalib() {
		if (recognizer) delete recognizer;
		recognizer = 0;
	}

	/**
	 * パラメータの設定及び必要な初期化を行います.
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] cc カメラキャリブパラメータ
	 * @param [in] cp キャリブボードのパラメータ
	 * @param [in] mp 円マーカ検出のためのパラメータ
	 * @param [in] pp マーカ検出のための前処理用パラメータ
	 */
	bool set_parameters(CameraCalibParams &cc, CalibBoardParams &cp, CircleMarkerParams &mp, PreProcParams &pp);

	/**
	 * 単体カメラキャリブレーションを行います.
	 * @return 内部パラメータの推定に失敗した場合はfalse. 成功した場合はtrue.
	 * @param [out] discam 歪み有カメラ
	 * @param [in] point2ds 様々な姿勢で置かれたキャリブボードのマーカ位置集合
	 * @param [in] param キャリブレーションパラメータ
	 * @param [out] rpjerr 各姿勢毎の再投影誤差
	 * @param [out] camerr 最終的な再投影誤差の総和(? cv::calibrateCameraの戻値)
	 * @warning 与えられたマーカ画像内位置と三次元位置のペアが各画像毎に4点以上ある場合に、その画像はキャリブレーションに使用される.
	 * この条件を満足する画像が一枚もなければキャリブレーションは失敗する.
	 */
	bool calib_intrinsic(DistortCamera &discam, std::vector< std::vector<cv::Point2f> > &point2ds,
		std::vector<double> &rpjerr, double *camerr = 0);

public:
	/// キャリブボード認識器
	iCalibBoardRecognizer* recognizer;

protected:
	/// キャリブレーションパラメータ
	CameraCalibParams calib_param;

private:
	/// キャリブレーションボードタイプ(0: YOODS, 1: AVS)
	int board_type;
};


/**
 * ステレオカメラキャリブレーションクラス
 */
class StereoCalib : public CameraCalib {
public:
	enum { N_CAMERA = 2 };
	StereoCalib(const int _type = 0) : CameraCalib(_type) {}
	virtual ~StereoCalib() {}

	/**
	 * ステレオカメラとしてキャリブレーションを行います.
	 * @return 外部パラメータの推定に失敗した場合はfalse. 成功した場合はtrue.
	 * @param [in|out] discam 歪み有カメラ(0: 左目カメラ, 1: 右目カメラ)
	 * @param [in] point2ds マーカ位置
	 * @param [in] cflags ステレオカメラキャリブレーションの為のフラグ値
	 * @param [out] camerr ステレオカメラとしての再投影誤差総和
	 */
	bool calib_extrinsic(std::vector<DistortCamera> &discam,
		std::vector< std::vector< std::vector<cv::Point2f> > > &point2ds,
		const int cflags = cv::CALIB_USE_INTRINSIC_GUESS, double *camerr = 0);

	/**
	 * カメラキャリブレーションを行います.
	 * @return 状態値(0: 成功, 1: 単体カメラキャリブレーションに失敗, 2: ステレオカメラキャリブレーションに失敗)
	 * @param [out] camera キャリブレーション対象のカメラ(歪み有カメラ)
	 * @param [in] point2ds マーカ位置
	 * @param [in] flags ステレオカメラキャリブレーションの為のフラグ値リスト
	 */
	int exec(std::vector<DistortCamera> &discam,
		std::vector< std::vector< std::vector<cv::Point2f> > > &point2ds,
		std::vector< std::vector<double> > &rpjerr, double *camerr = 0);
};

