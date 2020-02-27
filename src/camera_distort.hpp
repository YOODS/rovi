#pragma once
#include "camera_pinhole.hpp"


/**
 * 歪みのあるレンズを持つカメラ 
 */
class DistortCamera : public PinHoleCamera {
public:
	cv::Mat_<double> D;		///< 歪みパラメータ

public:
	/// コンストラクタ
	DistortCamera(cv::Size _size = cv::Size(0, 0), std::string _name = "camdis")
		: PinHoleCamera(_size, _name) {}

	
	/// デストラクタ
	virtual ~DistortCamera() {}

	
	/// コピーコンストラクタ
	DistortCamera(const DistortCamera &obj) : PinHoleCamera(obj) {
		this->D = obj.D.clone();		
	}
	

	/// 代入演算子
	DistortCamera& operator=(const DistortCamera &obj) {
		if (this != &obj) {
			PinHoleCamera::operator=(obj);
			this->D = obj.D.clone();
		}
		return (*this);		
	}

	/**
	 * カメラパラメータをファイルストレージに書き出す
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] fs ファイルストレージへのポインタ.0が指定された場合は、create_param_filename()
	 * で得られるファイル名で出力する。
	 */
	bool save(cv::FileStorage *_fs = 0);

	/**
	 * カメラパラメータをファイルストレージから読み込む。
	 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
	 * @param[in] fs ファイルストレージへのポインタ.0が指定された場合は、create_param_filename()
	 * で得られるファイル名で出力する。
	 */
	bool load(cv::FileStorage *_fs = 0);
	

	/**
	 * 与えられた三次元点をカメラ画像上に投影します.
	 * @return 投影された点の位置
	 * @param p 投影する点(三次元点)
	 */	
	cv::Point2d project_forw(cv::Point3d p);

	
	cv::Point3d project_back(cv::Point2d p);
	cv::Vec3d get_sight_vector(cv::Point2d p);

	bool solvePnPRansac(cv::Mat &rvec, cv::Mat &tvec, std::vector<cv::Point2f> &point2s, std::vector<cv::Point3f> &point3d);
	bool solvePnP(cv::Mat &rvec, cv::Mat &tvec, std::vector<cv::Point2f> &point2s, std::vector<cv::Point3f> &point3d);
};
