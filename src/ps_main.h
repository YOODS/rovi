//
//  ps_main.h
//  PhaseShift
//
//  Created by 原田 寛 on 2018/03/30.
//  Copyright © 2018年 原田 寛. All rights reserved.
//

#ifndef ps_main_h
#define ps_main_h

#include "common.h"
#include "camera.h"
#include "camera_calibration.h"
#include "phase_shift.h"

// 点群データ
extern PointCloud *_pcd;
extern int _pcn;

// 位相シフト計算クラス(Global)
extern PHASE_SHIFT ps;

// ステレオカメラクラス(Global)
extern StereoCamera stcam;

/*
 * カメラ,位相シフトを初期化する。
 * 指定したディレクトリにrect.paramがあれば、hmat0/1.dat,rect.paramを使う。
 * 無ければ、cam0/1_param.yamlから動的にhmat0/1.dat,rect.paramを作る。
 *
 * w: width, h: height
 * p: 位相シフトパラメータ
 * dirname: キャリブデータ格納ディレクトリ
 */
enum { CAM_COORD, BD_COORD};
extern void ps_init(int w,int h,PS_PARAMS& p,int coord,const char *dirname);
extern void ps_init(int w,int h);

/*
 * 位相シフトパラメータを更新する
 * p: 位相シフトパラメータ
 */
extern void ps_setparams(PS_PARAMS& p);

/*
 * 位相シフト撮影した写真データをセットする
 * cam: カメラ番号 左:CAM1(0), 右:CAM2(1)
 * idx: 0-12 (写真のインデックス)
 * pict: OpenCVのMat<CV_8UC1>形式
 */
extern void ps_setpict(int cam, int idx, cv::Mat &pict);

/*
 * 位相シフト実行
 * 出力: 視差マップ(double形式のmatrix)
 */
extern Eigen::MatrixXd& ps_exec(void);

/*
 * 点群生成
 * diff: 視差マップ(位相シフト,SGBM等で生成)
 * texture: 点群に紐づけるテクスチャ(レクティファイ済みの画像)
 * mask: 点群生成するかしないかのフラグマップ(0のみ点群生成)
 * pt: PointCloud[width,height]の配列
 *
 * [グローバルの_pcd, _pcnに点群データが入る]
 */
extern int genPC(Eigen::MatrixXd &diff,Eigen::MatrixXp &texture,Eigen::MatrixXp &mask,Eigen::MatrixXpt &pt);
extern int genPC(Eigen::MatrixXd &diff,Eigen::MatrixXp &texture,Eigen::MatrixXp &mask,Eigen::MatrixXpt &pt,Eigen::Matrix4d &Q);

/*
 * 生成した点群をPLY保存
 * fname: ファイル名(パス名を含む)
 */
extern void outPLY(const char *fname);


#endif /* ps_main_h */
