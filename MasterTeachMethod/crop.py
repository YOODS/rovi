#coding: utf-8

import numpy as np
import open3d as o3

def crop(crop_filename, data3d, verbose=False):
    '''
    @param crop_filename 3Dデータのクロップ設定ファイル名
    '''
    # クロップボリュームファイルを読み込む
    volume = o3.read_selection_polygon_volume(crop_filename)
    
    # PointCloud作成
    cloud = o3.PointCloud()
    cloud.points = o3.Vector3dVector(data3d)

    if verbose:
        print cloud, '\t<- Original'
        tpoints = np.asarray(cloud.points)
        print '\txrange=[{}, {}]'.format(np.min(tpoints[:, 0]), np.max(tpoints[:, 0]))
        print '\tyrange=[{}, {}]'.format(np.min(tpoints[:, 1]), np.max(tpoints[:, 1]))
        print '\tzrange=[{}, {}]'.format(np.min(tpoints[:, 2]), np.max(tpoints[:, 2]))
    
    # クロップ
    crpcl = volume.crop_point_cloud(cloud)
    if verbose:
        print 'SelectionPolygonVloume ->'
        print '\torthogonal_axis = ', volume.orthogonal_axis
        print '\taxis_min = ', volume.axis_min
        print '\taxis_max = ', volume.axis_max
        print '\tbounding_polygon = ', np.asarray(volume.bounding_polygon)            
        print crpcl, '\t<- Cropped'
    
        # デバッグ用に、オリジナル点群とクロップされた結果を同時に表示する
        cloud.paint_uniform_color([0.5, 0.5, 0.5])    # 元の点群を灰色で塗りつぶす
        crpcl.paint_uniform_color([1, 0, 0])    # クロップされた点群を赤で塗りつぶす
        o3.draw_geometries([cloud, crpcl])
        
        
    return np.asarray(crpcl.points)


if __name__ == '__main__':
    import argparse
    import copy
    import sys
    
    parser = argparse.ArgumentParser(
        description='クロップを行います',
        epilog='',
        add_help=True,
    )

    parser.add_argument('data3d', metavar='data3d', help='クロップ対象3Dデータファイル名')    
    parser.add_argument('crop_filename', metavar='cropfilename', help='クロップファイル名')
    parser.add_argument('-v', '--verbose', help='デバッグメッセージ＆画像を表示させます.', action='store_true')    
    
    args = parser.parse_args()

    # 3Dデータの読み込み
    cloud = o3.read_point_cloud(args.data3d)

    if len(np.asarray(cloud.points)) == 0:
        print args.data3d, ': file read failure'
        sys.exit(1)

    data3d = crop(args.crop_filename, np.asarray(cloud.points), verbose=True)
    
    # 3Dデータ保存
    o3.write_point_cloud('tmp.ply', rslcl)
    print '3d data saved at tmp.ply'
