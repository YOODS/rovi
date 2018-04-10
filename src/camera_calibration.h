#pragma once
#include "CircleCalibBoard.h"
#include "camera.h"


/**
 * カメラキャリブレーションを実行します
 * @return 再投影誤差
 * @param[out] camera 歪み有カメラ
 * @param[out] errors 再投影誤差格納用バッファ(0が指定されていたら、再投影誤差は計算しない)
 * @param[in] imagePoints 検出されたマーカ位置集合
 * @param[in] cboard YOODSキャリブボード読み取り器
*/
double cam_calibrate(DistortCamera &camera, std::vector<double> *errors, std::vector< std::vector<cv::Point2f> > &imagePoints, CircleCalibBoard &cboard);

/**
 * ステレオキャリブレーションを実行します
 * @return 再投影誤差
* @param[in] camera キャリブレーション済みのカメラパラメータ(0: 左カメラ, 1: 右カメラ)

* @param[out] stereo ステレオカメラパラメータ
* @param[in] imagePoints 画像データ点(0: 左カメラから得られたデータ. 1: 右カメラから得られたデータ)
* @param[in] cboard YOODSキャリブボード読み取り器
* @warning camera[n]から取得されたデータがimagePoints[n]となるようにしてください。
*/
double stereo_calibrate(DistortCamera camera[], std::vector< std::vector<cv::Point2f> > imagePoints[], CircleCalibBoard &cboard);


/**
* ステレオカメラを平行化します。
* @return なし.
* @param[in|out] stereo ステレオカメラ
* @param[in] oldcam ステレオカメラとしてキャリブレーション済みの歪み有カメラ
*/
void stereo_rectify(StereoCamera &stereo, DistortCamera oldcam[]);



/**
 * ステレオカメラの座標系をカメラ座標系から参照座標系に変換します.
 * @return なし.
 * @param [in|out] stereo ステレオカメラ
 * @param [in] refimPoints 検出されたマーカ位置(0:左カメラ, 1: 右カメラ)
 * @param [in] cboard YOODSキャリブボード読み取り器
 */
void convert_reference_coodinate_system(StereoCamera &stereo, std::vector<cv::Point2f> refimPoints[], CircleCalibBoard &cboard);
