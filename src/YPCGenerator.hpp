#pragma once

#include "iStereoCamera.hpp"
#include "iPointCloudGenerator.hpp"
#include <chrono>


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
	  点群生成器に渡すファイルのフルパスのリストを作成します.
	  @return 作成されたリスト
	  @param [in] dirname 画像が格納されているディレクトリ名
	  @param [in] file_ptn ファイルの拡張子(?)
	  @note 左右分離画像からリストを作成する際には、画像が左0, 右0, 左1, 右1, ... の順になるよう並べてください.
	  @note 連結画像は横並びで連結されている画像のみ有効です
	 */
	virtual std::vector<std::string> create_filelist(const char *dirname, const char *file_ptn) = 0;

	/**
	  経過時間を表示します.
	  @return なし
	 */
	virtual void print_elapsed() {}

public:
	/**
	  点群の座標変換行列をファイルから読み込みます.
	 */
	bool convert_coordinate(const char *filename) {
		if (!pcgen) return false;
		return pcgen->convert_coordinate(filename);
	}

	/**
	  カメラがHMatから構築されたか否かを返します.座標系変換済みのHMatからステレオカメラを構築している可能性があるため	 */
	const bool is_camera_from_hmat() const {
		return (this->camtype == CamParamType::HMat) ? true : false;
	}

	/**
	  画像ファイルから画像を読み込んで点群生成を行います.
	  @return 作成された点の数
	  @param [in] filenames 画像ファイルパス(点群生成に必要な枚数だけが格納されているようにしてください)
	  @param [in] is_interpo 補間を行うか否か
	  @param [in] callback 点群生成後に呼び出されるコールバック関数
	  @note filenames.size()で画像が連結されているか、そうでないかを判定します.
	 */
	int generate_pointcloud(std::vector<std::string> filenames, const bool is_interpo, PointCloudCallback *callback) {
		if (!load_images(filenames)) return false;
		return this->exec(is_interpo, callback);
	}

	/**
	  画像バッファから画像を取り出して点群生成を行います.
	  @return 作成された点の数
	  @param [in] buffers 画像左上端アドレスが枚数分格納されているベクタ
	  @param [in] is_interpo 補間を行うか否か
	  @param [in] callback 点群生成後に呼び出されるコールバック関数
	  @note 画像はカメラから取り込んだままのものを渡してください.(レクティファイしないで下さい)
	  @note 画像バッファの水平方向のバイト数は入力画像横幅と同じにしてください.
	  @note 左右分離画像を渡す場合は、左0, 右0, 左1, 右1, ... の順になるよう並べておいてください.
	  @note 左右連結画像は左側に左カメラ画像、右側に右カメラ画像となるように連結されていることを想定しています.
	  @note buffers.size()で画像が連結されているか、そうでないかを判定します.
	 */
	int generate_pointcloud(std::vector<unsigned char*> &buffers, const bool is_interpo, PointCloudCallback *callback) {
		if (!set_images(buffers)) return false;
		return this->exec(is_interpo, callback);
	
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
	  点群を生成します.
	  @return 作成された点の数
	  @param [in] callback 点群作成後に呼び出されるコールバック関数
	 */
	int exec(const bool is_interpo, PointCloudCallback *callback);

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

	std::chrono::system_clock::duration elapsed_disparity;	///< 視差計算にかかった時間
	std::chrono::system_clock::duration elapsed_genpcloud;	///< 点群計算にかかった時間
};



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
