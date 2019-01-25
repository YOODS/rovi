# coding: utf-8

import numpy as np
import open3d as o3
import copy
from sklearn import linear_model

import time


def draw_registration_result(source, target, transform):
    stemp = copy.deepcopy(source)
    stemp.paint_uniform_color([1, 0.706, 0])
    stemp.transform(transform)
    o3.draw_geometries([stemp, target])


class ICPEstimatorError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)
        
# 点群読み込みに失敗
class ReadPointCloudError(ICPEstimatorError): pass

# 点が一つも無い
class NoPointInCloudError(ICPEstimatorError): pass


    
'''
与えられた3Dデータの位置と向き(剛体変換)を推定します
'''
class ICPEstimator:
    def __init__(self, voxel_size=4, neighborpt=6, fitness_threshold=0.5, verbose=False):
        '''
        初期化
        @param voxel_size ボクセルサイズ
        @param neighborpt 半径2*voxel_sizeの球の中に含まれる点数の下限値. これより小さい場合、その点は削除される
        @param verbose デバッグメッセージを表示する(True)か否(False)か
        '''
        # verbose設定
        self.verbose = verbose

        # パラメータ色々
        self.voxel_size = voxel_size    		# ボクセルサイズ
        self.neighborpt = neighborpt    		# 近傍点数の閾値
	self.fitness_threshold = fitness_threshold	# ICPの一致率閾値

        if self.verbose:
            print 'voxel_size \t:', self.voxel_size
            print 'neighborpt \t:', self.neighborpt
            print 'fitness_threshold \t:', self.fitness_threshold
            
        

    def __call__(self, master_filename, target_filename):
        '''
        位置＆回転推定処理本体
        @param master_filename マスターデータファイル名
        @param target_filename ターゲットデータファイル名
        '''

	stt=time.time()        ### SW

        # 3Dデータ読み込み
        master_cloud = self._load3d(master_filename)
        target_cloud = self._load3d(target_filename)

	print '_load3d done.           elapsed={0}'.format(time.time()-stt)+'[sec]'  ### SW
        
        # 位置合わせのためのデータ(法線ベクトルとか)を準備
        master_down, master_fpfh = self._get_features(master_cloud)
        target_down, target_fpfh = self._get_features(target_cloud)

	print '_fpfh done.           elapsed={0}'.format(time.time()-stt)+'[sec]'  ### SW
        
        # RANSACで大まかな位置合わせ
        distance_threshold = 1.5 * self.voxel_size
        result_ransac = o3.registration_ransac_based_on_feature_matching(
            master_down,
            target_down,
            master_fpfh,
            target_fpfh,
            distance_threshold,
       #     o3.TransformationEstimationPointToPoint(True), 4,
            o3.TransformationEstimationPointToPoint(False), 4,
            [o3.CorrespondenceCheckerBasedOnDistance(distance_threshold),
             o3.CorrespondenceCheckerBasedOnEdgeLength(0.9)],
       #     o3.RANSACConvergenceCriteria(400000, 500)  #反復の最大数と検証ステップの最大数
            o3.RANSACConvergenceCriteria(40000, 500)	#反復の最大数と検証ステップの最大数
            )

	print '_ransac done.           elapsed={0}'.format(time.time()-stt)+'[sec]'  ### SW

        #if self.verbose is True:
        if self.verbose:
            draw_registration_result(master_down, target_down, result_ransac.transformation)


        # 詳細位置合わせ
        # ダウンサンプリング(1mm間隔にしてみる。。)
        down_master = o3.voxel_down_sample(master_cloud, 1)
        down_target = o3.voxel_down_sample(target_cloud, 1)

	print '_down_sampling done.           elapsed={0}'.format(time.time()-stt)+'[sec]'  ### SW

	#オリジナル
        #o3.estimate_normals(master_cloud, o3.KDTreeSearchParamHybrid(radius=self.voxel_size, max_nn = 30))
        #o3.estimate_normals(target_cloud, o3.KDTreeSearchParamHybrid(radius=self.voxel_size, max_nn = 30))
	#ダウンサンプリング
        o3.estimate_normals(down_master, o3.KDTreeSearchParamHybrid(radius=self.voxel_size, max_nn = 30))
        o3.estimate_normals(down_target, o3.KDTreeSearchParamHybrid(radius=self.voxel_size, max_nn = 30))

	print '_normals done.           elapsed={0}'.format(time.time()-stt)+'[sec]'  ### SW

        result_icp = o3.registration_icp(
        #    master_cloud,			#マスター
        #    target_cloud,			#ターゲット
            down_master,			#ダウンサンプリングしたマスター
            down_target,			#ダウンサンプリングしたターゲット
        #    0.4 * self.voxel_size,		
            1.0,				#ダウンサンプリングの場合には1mm間隔にしてるので、ここも1.0にした
        #    result_ransac.transformation, o3.TransformationEstimationPointToPoint(True))
            result_ransac.transformation, o3.TransformationEstimationPointToPlane())

	print '_icp done.           elapsed={0}'.format(time.time()-stt)+'[sec]'  ### SW

        ##### 一致率及による成否判定
        print "fitness {0:.6f} RMSE {1:.6f}".format(result_icp.fitness,result_icp.inlier_rmse)
        print "fitness_threshold:",self.fitness_threshold

	if float(result_icp.fitness) < self.fitness_threshold:
	    print '###### fitness Error'
	    return None

        #if self.verbose is True:
        if self.verbose:
            draw_registration_result(master_cloud, target_cloud, result_icp.transformation)
            
        
        return result_icp.transformation
       
    
    def _load3d(self, data_filename):
        '''
        3Dデータを読み込み、クロップして返します
        @param data_filename 3Dデータファイル名
        '''
        # 3Dデータの読み込み
        cloud = o3.read_point_cloud(data_filename)

        # データが無ければ以降の処理は行わない
        if len(np.asarray(cloud.points)) == 0: raise ReadPointCloudError(data_filename)
        
        if self.verbose:
            print cloud, '\t<- Original'
            tpoints = np.asarray(cloud.points)            
            print '\txrange=[{}, {}]'.format(np.min(tpoints[:, 0]), np.max(tpoints[:, 0]))
            print '\tyrange=[{}, {}]'.format(np.min(tpoints[:, 1]), np.max(tpoints[:, 1]))
            print '\tzrange=[{}, {}]'.format(np.min(tpoints[:, 2]), np.max(tpoints[:, 2]))
            o3.draw_geometries([cloud])
                
        return cloud
       


    def _get_features(self, cloud):
        # ダウンサンプリング
        down = o3.voxel_down_sample(cloud, self.voxel_size)
        if self.verbose:
            print down, '\t<- down sample'
        
        # 法線ベクトルを推定
        radius_n = 2 * self.voxel_size
        o3.estimate_normals(down, o3.KDTreeSearchParamHybrid(radius=radius_n, max_nn=30))

        # FPFH特徴を計算する
        radius_f = 5 * self.voxel_size
        fpfh = o3.compute_fpfh_feature(down, o3.KDTreeSearchParamHybrid(radius=radius_f, max_nn = 100))
        if self.verbose:
            print fpfh, '\t<- FPFH feature'
        
        return down, fpfh
                               
    
        
def CreateEstimator(voxel_size, neighbor, fitness_threshold, verbose):
    estimator = None
    try:
        estimator = ICPEstimator(voxel_size, neighbor, fitness_threshold, verbose)
    except Exception as e:
        print e
    finally:
        return estimator


    
def EstimateRT(estimator, master_3d, target_3d):
    RT = None
    try:
        RT = estimator(master_3d, target_3d)
    except ReadPointCloudError as e:
        print 'Read Point Cloud failure: ', e.value
    except NoPointInCloudError as e:
        print 'No Point in Cloud. check crop json file.'
    except IOError as e:
        print e, ' : No such file or directory'
    else:
        print 'Rigit Transform Matrix: \n', RT
    finally:
        return RT
        
        
        
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        description='マスターデータをターゲットデータに重ねるための剛体変換を推定します',
        epilog='',
        add_help=True,
    )

    parser.add_argument('master_3d', metavar='master_3d', help='マスター3Dデータファイル名')    
    parser.add_argument('target_3d', metavar='target_3d', help='ターゲット3Dデータファイル名')
    parser.add_argument('-s', '--voxel_size', help='ボクセルサイズ',
                        action='store', type=float, const=None, default=8.)
    parser.add_argument('-n', '--neighbor', help='ノイズ除去の閾値(点の数).ノイズ除去にはradius_outlier_removalを使用.半径は2*voxel_sizeであることに注意', action='store', type=int, const=None, default=6)
    parser.add_argument('-f', '--fitness_threshold', help='ICPの一致率閾値', action='store', type=float, const=None, default=0.5)
    parser.add_argument('-v', '--verbose', help='デバッグメッセージ＆画像を表示させます.', action='store_true')    

    
    args = parser.parse_args()

    # 初期化
    estimator = CreateEstimator(args.voxel_size, args.neighbor, args.verbose)

    # 物体の剛体変換を推定
    if estimator is not None:
        RT = EstimateRT(estimator, args.master_3d, args.target_3d)

    
    
