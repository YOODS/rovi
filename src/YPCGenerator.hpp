#pragma once

#include "iStereoCamera.hpp"
#include "iPointCloudGenerator.hpp"
#include <chrono>
#include <vector>


class YPCGenerator {
public:
	YPCGenerator()
		: stereo(0), settings(), pcgen(0), method3d(iPointCloudGenerator::Method3D::QMatrix) {
	}

	virtual ~YPCGenerator() {
		if (stereo) stereo->destroy();
		if (pcgen) pcgen->destroy();
	}

	/**
	  点群生成器を作成します.
	  @return 作成に成功した場合はtrue, 失敗した場合はfalse
	  @param [in] pcgen_mode 点群生成モード(iPointCloudGenerator.hppのPcGenModeを参照の事)
	 */
	virtual bool create_pcgen(const PcGenMode pcgen_mode) = 0;

	/**
	  点群生成パラメータの読み込みを行い、点群生成器にパラメータを渡します.
	  @return 読み込みに成功した場合はtrue, 失敗した場合はfalse.
	  @param [in] filename パラメータファイル名
	 */
	virtual bool init(const char *filename) = 0;

	/**
	  点群生成に必要なステレオカメラを作成します.
	  @return 作成に成功した場合はtrue, 失敗した場合はfalse.
	  @param [in] dirname カメラパラメータファイルが格納されているディレクトリ名
	 */
	virtual bool create_camera(const char *dirname) = 0;

	/**
	  カメラがHMatから構築されたか否かを返します.座標系変換済みのHMatからステレオカメラを構築している可能性があるため
	 */
	const bool is_camera_from_hmat() const {
		return (this->camtype == CamParamType::HMat) ? true : false;
	}

	/**
	  点群の座標変換行列をファイルから読み込みます.
	  @return 読み込みに成功した場合はtrue, 失敗した場合はfalse.
	  @param [in] filename 座標変換行列が記述されているファイル名
	 */
	bool convert_coordinate(const char *filename) {
		if (!pcgen) return false;
		return pcgen->convert_coordinate(filename);
	}

	/**
	  点群生成器に渡すファイルのフルパスのリストを作成します.
	  @return 作成されたリスト
	  @param [in] dirname 画像が格納されているディレクトリ名
	  @param [in] file_ptn ファイルの拡張子(?)
	  @note 左右分離画像からリストを作成する際には、画像が左0, 右0, 左1, 右1, ... の順になるよう並べてください.
	  @note 連結画像は横並びで連結されている画像のみ有効です
	 */
	virtual std::vector<std::string> create_filelist(const char *dirname, const char *file_ptn) = 0;

	/**
	 * 点群生成器に残っているデータをクリアします.点群生成前に一回必ず呼び出してください.
	 */
	bool reset();

	/**
	  ファイルから画像を読み込んで点群生成器に渡します.
	  @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	  @param [in] filenames ファイルのフルパスが格納されているvector
	 */
	bool load_images(std::vector<std::string> &filenames);

	/**
	  バッファから画像を点群生成器に渡します.
	  @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	  @param [in] buffers 画像左上端アドレスが格納されているvector
	 */
	bool set_images(std::vector<unsigned char*> &buffers);

	/**
	  経過時間を表示します.
	  @return なし
	 */
	virtual void print_elapsed() {}

public:
	/**
	 * 3Dマッチングを行う前に必要な前処理を行います.
	 * @return 処理が成功した場合はtrue, 失敗した場合はfalse.
	 */
	bool preprocess();

	/**
	  視差を求め、点群を生成します.
	  @return 処理が成功したか否か
	  @param [in] texture_cam 点群に張り付けるテクスチャをどちらのカメラからの画像を使用するか(0: 左(従来通り), 1: 右)
	 */
	bool execute(const int texture_cam);
	
	/**
	   点群を保存します.
	   @return 作成された点の数
	   @param [in] callback 点群生成後に呼び出されるコールバック関数
	 */
	int save_pointcloud(PointCloudCallback *callback) {
		return pcgen->get_pointcloud(callback);
	}

	// 下はexecuteとsave_pointcloudをまとめて行う関数
	// generate_pointcloud呼び出しの前に必ずload_images or set_imagesにて画像をpcgenに
	// 与え、preprocess()を呼び出しておかなければならない.

	/**
	  画像ファイルから画像を読み込んで点群生成を行います.
	  @return 作成された点の数
	  @param [in] texture_cam 点群に張り付けるテクスチャをどちらのカメラからの画像を使用するか(0: 左(従来通り), 1: 右)
	  @param [in] callback 点群生成後に呼び出されるコールバック関数
	 */
	int generate_pointcloud(const int texture_cam, PointCloudCallback *callback) {
		if (!this->execute(texture_cam)) return false;
		return this->save_pointcloud(callback);
	}

protected:
	/**
	  リマップ後の画像横幅を返します
	  @return リマップ後の画像横幅
	 */
	const int get_image_cols(void) const { return settings.output_cols; }

	/**
	  リマップ後の画像縦幅を返します
	  @return リマップ後の画像縦幅
	 */
	const int get_image_rows(void) const { return settings.output_rows; }

	/**
	  カメラ画像の横幅を設定します
	  @return なし
	  @param [in] wd カメラ画像横幅
	 */
	void set_camera_cols(const int cols) {
		settings.input_cols = cols;

		// 今の所、レクティファイ後の画像サイズ == 入力画像サイズとしてしか運用していないので
		// 出力画像サイズも同じ値を設定する.が、違う値も設定できるので忘れないように...
		settings.output_cols = cols;
	}

	/**
	  カメラ画像の縦幅を設定します
	  @return なし
	  @param [in] ht カメラ画像縦幅
	 */
	void set_camera_rows(const int rows) {
		settings.input_rows = rows;

		// 今の所、レクティファイ後の画像サイズ == 入力画像サイズとしてしか運用していないので
		// 出力画像サイズも同じ値を設定する.が、違う値も設定できるので忘れないように...
		settings.output_rows = rows;
	}

protected:
	/// カメラタイプ
	CamParamType camtype;

	/// ステレオカメラ
	iStereoCamera *stereo;

	/// ステレオカメラ作成のための設定
	StereoCameraSettings settings;

	/// 点群生成器
	iPointCloudGenerator *pcgen;

	/// 点群計算方法
	iPointCloudGenerator::Method3D method3d;

	std::chrono::system_clock::duration elapsed_phsdecode;	///< 位相復号
	std::chrono::system_clock::duration elapsed_preprocess;	///< 前処理にかかった時間(位相接続チェック＆レクティファイ)
	std::chrono::system_clock::duration elapsed_makedisp;	///< 視差計算にかかった時間
	std::chrono::system_clock::duration elapsed_genpcloud;	///< 点群計算にかかった時間

	void time_start() { time_beg = std::chrono::system_clock::now(); }
	std::chrono::system_clock::duration get_elapsed() { 
		std::chrono::system_clock::time_point time_end = std::chrono::system_clock::now();	
		return std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_beg);
	}
	std::chrono::system_clock::time_point time_beg;
};



// PLY ファイルに点群データを保存するcallback
class PLYSaver : public PointCloudCallback {
	std::string filename;
	bool status;

public:
	PLYSaver(const std::string name) : filename(name) {}

	const bool is_ok() const { return status; }
	const std::string get_filename() const { return filename; }

	/**
	  get_pointcloudに渡したときに呼び出されるcallback関数(PLYに保存します. 無効な点は除外します.)
	  @note 保存に成功したか否かはstatusに保存されます.
	 */
	void operator()(unsigned char *image, const size_t step, const int width, const int height,
		std::vector<Point3d> &points, const int n_valid);
};


// Depthデータを保存するcallback
class DepthSaver : public PointCloudCallback {
	std::string filename;
	bool status;

public:
	DepthSaver(const std::string name) : filename(name) {}

	const bool is_ok() const { return status; }
	const std::string get_filename() const { return filename; }

	/**
	   get_pointcloudに渡したときに呼び出されるcallback関数(Depthデータ(最小-最大で正規化)を保存します.)
	   @note 保存に成功したか否かはstatusに保存されます.
	 */
	void operator()(unsigned char *image, const size_t step, const int width, const int height,
		std::vector<Point3d> &points, const int n_valid);
};	
