#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "CalibBoard.h"
#include "ParaConfigure.h"


class CircleCalibBoard : public CalibBoard {
public:
	enum Status {
		Success = 0,
		MemAlloc,			///< メモリの確保に失敗した
		ImageSizeChange,	///< 画像サイズが変わった
		TooFewContours,		///< 輪郭線の数が少なすぎる
		BaseMarkerNotFound,	///< 基準マーカーが見つからない
		InvalidPosition,	///< 見つかったマーカーの位置が不正(正しく読み取られなかった)
	};


	CircleCalibBoard() : CalibBoard(), image_h_size(0), image_v_size(0) {
		para["n_circles_x"] = 13;		///< X軸方向の円の数
		para["n_circles_y"] = 19;		///< Y軸方向の円の数
		para["origin_x"] = 6;			///< 原点位置のX成分(X軸プラス方向から0スタートで数えた番号)
		para["origin_y"] = 9;			///< 原点位置のY成分(Y軸プラス方向から0スタートで数えた番号)
		para["unitleng"] = 60.0;		///< マーカー重心間の距離
		para["distance_between_circles"] = 1.2;	///< 次の円の開始位置までの距離(円の直径に対する比率)

		para["n_circles_minimum"] = 9;	///< 最小必要輪郭数(4より小さくしないでください)
		para["min_rate"] = 0.25;	///< キャリブ板の最小サイズ(画面サイズに対する比率)
		para["max_rate"] = 2.00;	///< キャリブ板の最大サイズ(画面サイズに対する比率)
		para["fitscore"] = 0.95;	///< 検出された輪郭線を楕円近似した際に、輪郭線上の点がどの程度楕円上に乗っていれば輪郭線=楕円とするか
		para["fitrange"] = 1.50;	///< 近似された楕円とデータとの許容誤差
	}


	~CircleCalibBoard() {
		world_position.release();
	}


	/**
	 * 初期化を行います. scan()を呼び出す前に必ず一度は呼び出してください.
	 * @return なし
	 */
	void init();


	/**
	 * 与えられた画像から特徴点の座標値を見つけ出します.
	 * @param image キャリブレーションボード画像
	 * @param points 画像座標値画像(para["n_circles_x"] × para["n_circles_y"])
	 * @param dispimg デバッグ用結果表示画像
	 */
	int scan(cv::Mat &image, std::vector<cv::Point2f> &imgpoints, cv::Mat *dispimg = 0);


	/**
	 * 指定された位置にあるマーカのワールド座標値を返します.
	 * @return ワールド座標値
	 * @param [in] i 位置のX座標値
	 * @param [in] j 位置のY座標値
	 */
	cv::Point3f get_3d_position(const int i, const int j) {
		return world_position[j][i];
	}

	/**
	 * 指定された位置にあるマーカのワールド座標値を返します.
	 * @return ワールド座標値
	 * @param [in] n 先頭からの格納順
	 */
	cv::Point3f get_3d_position(const int n) {
		const int cols = (int)para["n_circles_x"];
		return world_position[n / cols][n % cols];
	}

	/**
	 * キャリブレーションに使用した入力画像サイズを返します.
	 * @return 入力画像サイズ
	 */
	cv::Size get_image_size() const {
		return cv::Size((int) image_h_size, (int) image_v_size);
	}

	/**
	 * 原点マーカが格納されているインデックスを返します.
	 * @return 原点マーカが格納されているインデックス
	 */
	size_t get_origin_marker_index() {
		return origin_pos.x + origin_pos.y * ((int)para["n_circles_x"]);
	}

	/**
	 * X軸マーカが格納されているインデックスを返します.
	 * @return X軸マーカが格納されているインデックス
	 */
	size_t get_x_axis_marker_index() {
		return x_axis_pos.x + x_axis_pos.y * ((int)para["n_circles_x"]);
	}

	/**
	 * Y軸マーカが格納されているインデックスを返します.
	 * @return Y軸マーカが格納されているインデックス
	 */
	size_t get_y_axis_marker_index() {
		return y_axis_pos.x + y_axis_pos.y * ((int)para["n_circles_x"]);
	}


private:
	size_t image_h_size;	///< 画像横幅
	size_t image_v_size;	///< 画像縦幅

	size_t min_length;	///< マーカー輪郭線長の下限値
	size_t max_length;	///< マーカー輪郭線長の上限値

	size_t min_radius;	///< マーカー円半径の下限値
	size_t max_radius;	///< マーカー円半径の上限値

	cv::Mat_<cv::Point3f> world_position;	///< ワールド座標値(キャリブレーション板のプラスの方向からマイナスの方向に向かって並べられている(ややこしい))
	std::vector<cv::Point> marker_position;	///< 原点マーカを基準としたマーカ位置(画像座標位置を決定する順番に並べ替えられる)
	cv::Point origin_pos;	///< マーカー原点のキャリブ板左上端マーカからの位置
	cv::Point x_axis_pos;	///< X軸方向点のキャリブ板左上端マーカからの位置
	cv::Point y_axis_pos;	///< Y軸方向点のキャリブ板左上端マーカからの位置


private:
	/**
	 * マーカー検出のための制限値を画像サイズから計算する
	 * @return 失敗したらfalse, 成功したらtrue.
	 * @param image_h_size 画像横幅
	 * @param image_v_size 画像縦幅
	 */
	bool set_limit(const size_t image_h_size, const size_t image_v_size);


	/**
	 * 与えられた画像サイズと、それに写っているキャリブ板の大体の比率(画像サイズに対する)から
	 * キャリブマーカーの円の半径と周囲長を計算します.
	 * @return なし
	 * @param radius 計算された半径
	 * @param length 計算された円周
	 * @param image_size 画像サイズ
	 * @param n_circles マーカーの数(画像サイズに与えられた大きさの方向での)
	 * @param rate キャリブ板の大きさの画像サイズに対する比率
	 */
	void calc_circle_limit(size_t &radius, size_t &length, const size_t image_size, const size_t n_circles, const float rate);


	/**
	 * 取得された輪郭線から楕円以外を取り除きます.
	 * @return 楕円半径の中央値を返します(水平、垂直込みの). 処理が失敗した場合は0を返します.
	 * @param contours 輪郭線集合
	 * @param centers 輪郭線の重心座標
	 */
	float remove_not_elliptical_contour(std::vector< std::vector<cv::Point> > &contours, std::vector<cv::Point2f> &centers);


	/**
	 * 輪郭線集合を近いもの同士でまとめてグループ分けし、最大の要素を持つグループの輪郭線のみを残して他を消去します.
	 * @return なし
	 * @param contours 輪郭線集合
	 * @param centers 輪郭線の重心
	 * @param threshold 重心間距離の閾値
	 */
	void leave_largest_group(std::vector< std::vector<cv::Point> > &contours, std::vector<cv::Point2f> &centers, float threshold);


	/**
	 * 基準マーカーの位置を探します.
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param base0 基準マーカー原点
	 * @param base1 基準マーカー水平方向点
	 * @param base2 基準マーカー垂直方向点
	 */
	bool search_base_marker_position(cv::Point2f &base0, cv::Point2f &base1, cv::Point2f &base2, std::vector<cv::Point2f> &centers, std::vector< std::vector<cv::Point> > &contours);
};

