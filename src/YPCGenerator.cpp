#include "YPCGenerator.hpp"
#include <fstream>
#include <cmath>
#include <cstdio>

bool YPCGenerator::load_images(std::vector<std::string>& filenames)
{
	if (filenames.size() == pcgen->requiredframes()) {
		// 連結画像の場合
		for (int n = 0; n < ((int)pcgen->requiredframes()); n++) {
			fprintf(stderr, "%d: %s\n", n, filenames[n].c_str());
			if (!pcgen->loadpict(filenames[n], 2, n)) return false;
		}
	}
	else if (filenames.size() == 2 * ((int)pcgen->requiredframes())) {
		// 分離画像の場合
		for (int n = 0, c = 0; n < pcgen->requiredframes(); n++, c += 2) {
			if (!pcgen->loadpict(filenames[c    ], 0, n)) return false;
			if (!pcgen->loadpict(filenames[c + 1], 1, n)) return false;
		}
	}
	else return false;
	return true;
}


bool YPCGenerator::set_images(std::vector<unsigned char*>& buffers)
{
	if (buffers.size() == pcgen->requiredframes()) {
		// 連結画像の場合
		for (size_t n = 0; n < pcgen->requiredframes(); n++) {
			if (!pcgen->setpict(buffers[n], settings.input_cols, 2, n)) return false;
		}
	}
	else if (buffers.size() == 2 * pcgen->requiredframes()) {
		// 分離画像の場合
		for (size_t n = 0, c = 0; n < pcgen->requiredframes(); n++, c += 2) {
			if (!pcgen->setpict(buffers[c    ], settings.input_cols, 0, n)) return false;
			if (!pcgen->setpict(buffers[c + 1], settings.input_cols, 1, n)) return false;
		}
	}
	else return false;
	return true;
}


bool YPCGenerator::execute(const bool is_interpo)
{
	this->elapsed_disparity = std::chrono::duration_values<std::chrono::microseconds>::zero();
	this->elapsed_genpcloud = std::chrono::duration_values<std::chrono::microseconds>::zero();
	if (!this->pcgen) return false;	// 点群生成器がなければ終了

	// 視差マップ作成
	std::chrono::system_clock::time_point beg, end;
	beg = std::chrono::system_clock::now();
	if (!pcgen->exec()) return false;
	end = std::chrono::system_clock::now();
	this->elapsed_disparity = std::chrono::duration_cast<std::chrono::microseconds>(end - beg);

	// 点群作成
	beg = std::chrono::system_clock::now();
	pcgen->genPC(this->method3d, is_interpo);
	end = std::chrono::system_clock::now();
	this->elapsed_genpcloud = std::chrono::duration_cast<std::chrono::microseconds>(end - beg);

	// 経過時間表示
	print_elapsed();
	return true;
}





void PLYSaver::operator()(unsigned char * image, const size_t step, const int width, const int height, std::vector<Point3d>& points, const int n_valid)
{
	std::ofstream ofs(filename, std::ios::binary);
	if (!ofs.is_open()) {
		status = false;
		return;
	}

	ofs << "ply\n";
	ofs << "format binary_little_endian 1.0\n";
	ofs << "comment VCGLIB generated\n";
	ofs << "element vertex " << points.size() << std::endl;
	ofs << "property float x\n";
	ofs << "property float y\n";
	ofs << "property float z\n";
	ofs << "property uchar red\n";
	ofs << "property uchar green\n";
	ofs << "property uchar blue\n";
	ofs << "element face 0\n";
	ofs << "end_header\n";

	for (int j = 0, n = 0; j < height; j++) {
		unsigned char *iP = image;
		for (int i = 0; i < width; i++, n++) {
			float pos[3] = {0, 0, 0};
			unsigned char col[3] = {iP[i], iP[i], iP[i]};
				
			if (!std::isnan(points[n].x)) {
				pos[0] = points[n].x;
				pos[1] = points[n].y;
				pos[2] = points[n].z;
			}

			ofs.write((char *)pos, sizeof(float) * 3);
			ofs.write((char *)col, 3);
		}
		image += step;
	}

	ofs.close();
	status = true;
}


#include <opencv2/opencv.hpp>
void DepthSaver::operator()(unsigned char *image, const size_t step, const int width, const int height, std::vector<Point3d>& points, const int n_valid)
{
	// 奥行きの最小最大を求める
	float min_ = std::numeric_limits<float>::max();
	float max_ = -min_;
	for (auto p : points) {
		if (std::isnan(p.x)) continue;
		if (min_ > p.z) min_ = p.z;
		if (max_ < p.z) max_ = p.z;
	}

	// 正規化用の係数を求める. 255は、測定できなかった点に入れる
	float scale = 254.0 / (max_ - min_);
	
	cv::Mat dmapim(height, width, CV_8U);
	for (int j = 0, n = 0; j < height; j++) {
		uchar *dP = dmapim.ptr<uchar>(j);
		for (int i = 0; i < width; i++, n++) {
			if (std::isnan(points[n].x)) {
				dP[i] = 255;
				continue;
			}
			
			dP[i] = ((int) (scale * (points[n].z - min_) + 0.5f));
		}
	}

	cv::imwrite(filename, dmapim);
	status = true;	
}


