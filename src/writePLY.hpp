#pragma once

#include <string>
#include "iPointCloudGenerator.hpp"

/**
 * 点群データをPLY形式でファイルに保存します.
 * @return 処理に成功した場合はtrue, 失敗した場合はfalse.
 * @param [in] filename 出力先ファイル名
 * @param [in] cloud 点群データバッファ先頭アドレス
 * @param [in] n_point 総点数
 * @param [in] rgrid レンジグリッドデータバッファ先頭アドレス(出力しない場合は0)
 * @param [in] width レンジグリッドデータの横幅
 * @param [in] height レンジグリッドデータの縦幅
 */
bool writePLY(const std::string filename, const PointCloudElement *cloud, const int n_point,
	const unsigned int *rgrid = 0, const int width = 0, const int height = 0);
