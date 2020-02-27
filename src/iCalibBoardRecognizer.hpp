/*
 * @file iCalibBoardRecognizer.hpp
 * @breif キャリブボード認識器のインターフェース 
 * @date 2019/09/21
 *
 * @note キャリブボードは, 「〇マーカ」、「グリッド状にマーカが配置」、「基準マーカは二重丸」の条件を満たしていると
 * 仮定している. それ以外は今のところ想定していない.
 * @note キャリブボードは横長の矩形で、長辺の左方向がY軸の正の向き、短辺の上方向がX軸の正の向きとする。
 * @note 原点位置は必ず基準マーカとして与えられていなければならない.
 */

#pragma once

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#include <math.h>
#endif	// _USE_MATH_DEFINES

#include <opencv2/opencv.hpp>
#include <map>

/* 必要なパラメータとそのデフォルト値 */

 /// マーカ検出のための前処理用パラメータ
struct PreProcParams {
	bool do_reverse_bw;		///< 白黒反転を行うか？
	bool do_equalize_hist;	///< ヒストグラム均一化を行うか?
	bool do_smoothing;		///< スムージングを行うか?
	int bin_type;	///< 二値化タイプ(0: 通常二値化, 1: 判別分析二値化, 2: 適応二値化)
	int bin_param0;	///< 二値化閾値0(bin_type==0の場合閾値, bin_type==2の場合ブロックサイズ, その他の場合は使用しない)
	int bin_param1;	///< 二値化閾値1(bin_type==2の場合平均値からのオフセット値, その他の場合は使われない)
	double gamma_correction;	///< gamma補正値

	/// コンストラクタ. デフォルト値を設定
	PreProcParams() :
		do_reverse_bw(false),
		do_equalize_hist(false),
		do_smoothing(false),
		bin_type(1),
		bin_param0(0),
		bin_param1(0),
		gamma_correction(1.0)
	{}

	/// パラメータ辞書から値をセットする.キーの名前はメンバ変数と同じ
	void set(std::map<std::string, double> &params);
};


/// 円マーカ検出のためのパラメータ
struct CircleMarkerParams {
public:
	// 外から指定して貰わないとならないが、別にデフォルト値でも問題ないと思われるパラメータ
	double fitscore;	///< 輪郭線から近似された楕円上に乗っている輪郭線点の割合の下限値(これより低ければその輪郭線は楕円ではないと判断する)
	int n_minimum;		///< 最低限これ以上は見つかってくれなければ困るマーカの数

	// 外から指定して貰わないとどうしようもないパラメータだけどなくても良い(デバッグ表示するときだけ必要)
	int image_width;	///< 画像横幅
	int image_height;	///< 画像縦幅
	double debug_show_scale;	///< デバッグ画像を表示するときに何倍で表示するか(0.0にしておくと表示しない)

	// 画像のサイズが分かれば、大体決めることができるパラメータ
	double max_radius;	///< 近似楕円の長半径の上限値(必ず零以上の値)

	// 画像の状態によって決まるパラメータ.(丸の大きさによって変わる)
	double min_radius;	///< 近似楕円の短半径の下限値(必ず零以上の値)

public:
	/**
	 * コンストラクタ. デフォルト値を設定
	 */
	CircleMarkerParams() :
		fitscore(0.9),
		n_minimum(9),
		debug_show_scale(0.0), image_width(0), image_height(0),
		min_radius(8), max_radius(1500) {}

	/**
	 * 画像のサイズから最大値の推定値を作成
	 * @param [in] w 画像横幅
	 * @param [in] h 画像縦幅
	 */
	void estimate_maxradius(const int w, const int h) {
		const int leng = (w > h) ? w : h;

		// 画面いっぱいにn_minimum個のマーカが写っていると仮定すると、最も大きいマーカの半径が決められる
		this->max_radius = leng / (2.0 * sqrt((double)this->n_minimum));

		// 画像サイズを取っておく
		image_width = w;
		image_height = h;
	}

	/// パラメータ辞書から値をセットする.キーの名前はメンバ変数と同じ
	void set(std::map<std::string, double> &params);
};

/// キャリブボードのパラメータ
struct CalibBoardParams {
	double unit_length;					///< 円マーカ重心間距離

	int n_circles_x;	///< X軸方向のマーカ数
	int n_circles_y;	///< Y軸方向のマーカ数
	int origin_x;	///< X軸の＋方向から原点までのマーカ数(0スタートで数える)
	int origin_y;	///< Y軸の＋方向から原点までのマーカ数(0スタートで数える)

	double distance_between_circles;	///< 重心間距離の円の直径に対する比率(小数点第一位まで有効)

	// コンストラクタ. デフォルト値を設定
	CalibBoardParams() :
		unit_length(0.0), n_circles_x(0), n_circles_y(0), origin_x(0), origin_y(0),
		distance_between_circles(1.2)
	{}

	/// パラメータ辞書から値をセットする.キーの名前はメンバ変数と同じ
	void set(std::map<std::string, double> &paramlist);
};



/// キャリブボード認識器のインターフェース
class iCalibBoardRecognizer {
public:
	virtual void destroy() = 0;
	virtual ~iCalibBoardRecognizer() {}

	/**
	 * パラメータを設定します.
	 * @return パラメータ設定に問題があればfalse, 無ければtrue.
	 * @param [in] pp マーカ検出のための前処理用パラメータ
	 * @param [in] mp 円マーカ検出のためのパラメータ
	 * @param [in] cp キャリブボードのパラメータ
	 */
	virtual bool set_parameters(PreProcParams &pp, CircleMarkerParams &mp, CalibBoardParams &cp) = 0;

	/**
	 * マーカの位置関係を識別します.
	 * @return 状態値(成功した場合はゼロ, 失敗した場合は非ゼロのエラー番号)
	 * @param [in] image 処理対象画像
	 * @param [out] point2d マーカの画像内座標値が格納されてるバッファ
	 */
	virtual int recognize(cv::Mat &image, std::vector<cv::Point2f> &point2d) = 0;

	/**
	 * 上記の状態値のエラーメッセージの名前を返します.
	 * @return エラーメッセージ
	 * @param [in] status 状態値
	 */
	virtual std::string get_error_name(const int status) = 0;

	/**
	 * 処理対象画像の画像サイズを返します.
	 * @return 処理対象画像の画像サイズ
	 * @warning 但し、一枚でもマーカ検出を行った後でなければ有効にならない
	 */
	virtual cv::Size image_size() = 0;

	/**
	 * マーカ座標の範囲を返します
	 * @return マーカ座標の範囲(cv::Rect(X軸方向の最小値、Y軸方向の最小値, Xの範囲, Yの範囲)
	 */
	virtual const cv::Rect get_marker_range() const = 0;

	/**
	 * マーカの縦横の位置から、その座標値が格納されているバッファにおけるインデックスを返します.
	 * @return インデックス値
	 * @param [in] pos マーカの(x,y)位置
	 */
	virtual const int get_marker_index(cv::Point pos) const = 0;

	/**
	 * 指定されたインデックスに格納されている画像座標値に対応するボードの三次元位置を返します.
	 * @return ボードの三次元位置
	 * @param [in] n インデックス
	 */
	virtual cv::Point3f get_3d_position(const int n) const = 0;
		
	/**
	 * 画像上でのマーカ位置が取得出来ている点だけを取り出して、imgPointsに保存する。同じ順序になるよう
	 * にobjPointsにその三次元位置を格納する.
	 * @return 位置が取得されたマーカ数
	 * @param [in] point2d マーカ位置
	 * @param [out] imgPoints 上記のマーカ位置の内、読み取れた位置のみ取り出したもの
	 * @param [out] objPoints imgPoints[i]に対応する三次元位置
	 */
	virtual size_t corresponding_points(const std::vector<cv::Point2f> &point2d,
		std::vector<cv::Point2f> &imgPoints,
		std::vector<cv::Point3f> &objPoints) = 0;

	/**
	 * 左右画像上でのマーカ位置が取得出来ている点だけを取り出して、imgPointsL, imgPointsRに保存する。
	 * 同じ順序になるように、pobjPointsにその三次元位置を格納する.
	 * @return 位置が取得されたマーカ数
	 * @param [in] point2dL 左カメラのマーカ位置
	 * @param [in] point2dR 右カメラのマーカ位置
	 * @param [out] imgPointsL 左カメラマーカ位置の内、読み取れた位置のみ取り出したもの
	 * @param [out] imgPointsR 右カメラマーカ位置の内、読み取れた位置のみ取り出したもの
	 * @param [out] objPoints imgPoints*[i]に対応する三次元位置
	 */
	virtual size_t corresponding_points(const std::vector<cv::Point2f> &point2dL, const std::vector<cv::Point2f> &point2dR,
		std::vector<cv::Point2f> &imgPointsL,
		std::vector<cv::Point2f> &imgPointsR,
		std::vector<cv::Point3f> &objPoints) = 0;
	
	/**
	 * マーカ識別結果画像をファイルに保存します.
	 * @return なし
	 * @parma [in] filename 保存先ファイル名
	 * @param [in] ret iCalibBoardRecognizer::recognize()の戻り値
	 */
	virtual void save_result_image(std::string filename, const int ret = 0) = 0;


	/**
	 * マーカ識別結果画像をコピーします.
	 * @return なし
	 * @param [in] image コピー先画像
	 * @param [in] ret iCalibBoardRecognizer::recognize()の戻り値
	 */
	virtual void copy_result_image(cv::Mat &image, const int ret = 0) = 0;
	
	/**
	 * マーカ識別結果画像を画面に表示します.
	 * @return なし
	 * @param [in] ret iCalibBoardRecognizer::recognize()の戻り値
	 */
	virtual void show_result_image(const int ret) = 0;
};


#undef _WINDOWS
#ifdef _WINDOWS
#ifdef _WINDLL
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __declspec(dllimport)
#endif
#else
#define EXPORT
#endif


/**
 * キャリブレーションボード認識器クラスインスタンス作成
 * @return クラスインスタンスのアドレス
 */
extern "C" EXPORT iCalibBoardRecognizer* CreateCalibBoardRecognizer();
