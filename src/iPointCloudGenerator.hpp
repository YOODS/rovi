#pragma once

/**
 * 点型
 */
struct PointCloudElement {
	float coord[3];			///< 座標値(x,y,zの順)
	unsigned char col[4];	///< 色(r,g,bの順. 但し今のところ白黒画像にしか対応していないので全て同じ値が格納される)
};

class iPointCloudGenerator {
public:
	/**
	 * 点群生成器を破棄します.
	 * @return なし
	 */
	virtual void destroy() = 0;

	/**
	 * 点群生成器の初期化を行います.(必要なバッファを確保します.)
	 * @return 成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] w 入力画像の横幅
	 * @param [in] h 入力画像の縦幅
	 */
	virtual bool init(const int w, const int h) = 0;

	/**
	 * 3D座標値計算に必要なカメラパラメータを指定されたディレクトリから読み取ります.
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] dirname カメラパラメータが格納されているディレクトリ名
	 * @warning こちらを呼び出した場合は、setpictで与えられた画像は未平行化画像として、内部で平行化が行われます.
	 */
	virtual bool set_camera_params(const char *dirname) = 0;

	/**
	 * 3D座標値計算に必要な透視変換行列(4x4)をセットします.
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param [in] Qbuff 透視変換行列がZスキャン順に格納されているバッファの先頭アドレス
	 * @warning こちらを呼び出した場合は、setpictで与えられた画像は平行化済画像として、そのまま処理対象画像として扱います.
	 */
	virtual bool set_camera_params(const double *QBuff) = 0;

	/**
	 * 点群生成に必要なパラメータをセットします.
	 * @param [in] params パラメータ型へのポインタ
	 * @warning params は手法によって異なります.位相シフトの場合はParamPSFT.hppを、SGBMの場合はParamSGBM.hppを参照してください.
	 */
	virtual void setparams(void *params) = 0;

	/**
	 * 処理対象画像をレクティファイします.
	 * @return なし
	 * @param [in] grayim 画像バッファ左上端アドレス
	 * @param [in] step 画像バッファの水平方向のバイト数
	 * @param [in] cam カメラ番号
	 * @warning ステレオカメラを作成した場合のみ使用可能になります. 作成していないのにも関わらず、呼び出した場合は何もしません.
	 */
	virtual void rectify(unsigned char *grayim, const int cols, const int rows, const size_t step, const int cam) = 0;

	/**
	 * 処理対象画像を生成器に渡します.
	 * @return なし.
	 * @param [in] grayim 画像バッファ左上端アドレス
	 * @param [in] step 画像バッファの水平方向のバイト数
	 * @param [in] cam カメラ番号
	 * @param [in] idx 画像番号(位相シフトの場合必ず必要)
	 */
	virtual void setpict(unsigned char *grayim, const size_t step, const int cam, const int idx = 0) = 0;

	/**
	 * 視差を計算します.
	 * @return なし.
	 */
	virtual void exec() = 0;

	/**
	 * 点群を作成します.
	 * @return 生成された点の数
	 * @param [in] method 三次元座標値を計算する手法(0: Q行列, 1: 視線ベクトル, 2: P行列)
	 * @warning method==1 or 2はステレオカメラを作成しないと使用できません.
	 */
	virtual int genPC(const int method = 0) = 0;

	/**
	 * 生成された点の数を返します.(genPC()の戻り値と同じ)
	 * @return 生成された点の数
	 */
	virtual const int get_pointcloud_cnt() const = 0;

	/**
	 * 生成された点データが格納されているバッファの先頭アドレスを返します.
	 * @return 点群データバッファの先頭アドレス
	 */
	virtual const PointCloudElement* get_pointcloud_ptr() const = 0;

	/**
	 * 生成された点群のレンジグリッドデータが格納されているバッファの先頭アドレスを返します.
	 * @return レンジグリッドバッファの先頭アドレス
	 * @warning サイズは画像サイズと同じです.
	 */
	virtual const unsigned int *get_rangegrid() const = 0;
};


/**
 * 点群生成器を作成します.
 * @return 点群生成器インスタンスへのポインタ
 * @param [in] method 点群生成手法(0: 位相シフト, 1: SGBM)
 */
iPointCloudGenerator* createPointCloudGenerator(const int method = 0);