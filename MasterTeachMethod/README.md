# Master Teaching


## 事前準備
pipが古いとopen3d-pythonのインストールでエラーとなるので、事前にアップグレードする。

```
python -m pip install --upgrade pip
```

追加パッケージ

```
sudo python -m pip install open3d-python
sudo python -m pip install scikit-learn 
npm install date_utils
```

以下、インストールされていなければ入れておく

~~~
sudo apt install ros-kinetic-jsk*
npm install mathjs
~~~

## パラメータ

パラメータファイル

```
cd ~/catkin_ws/src/rovi/MasterTeachMethod/
vi param.yaml
```

以下のパラメータを設定できる。

```
#デバッグファイル出力有無 1:デバッグファイル出力あり 0:デバッグファイル出力なし
debug: 1
#icp処理時のデバッグウィンドウ表示有無 1:ウィンドウ表示あり 0:ウィンドウ表示なし
verbose: 0
#cropファイルパス（点群のクロップ条件を指定するファイル）
crop_file_path: '/home/catkin_ws/src/rovi/MasterTeachMethod/sample_crop.json' 
#マスターデータ保存ディレクトリ
master_dir: '/home/catkin_ws/src/rovi/MasterTeachMethod/master/'
#デバッグファイル保存ディレクトリ
debug_data_dir: '/home/catkin_ws/src/rovi/MasterTeachMethod/debug_data/'
#icp一致率閾値
fitness_threshold: 0.5
#ロボット種別 Mitsubishi:三菱 Funac:Funac
robot_type: 'Mitsubishi'
```

クロップ条件設定ファイル（上記の例ではsample_crop.json）

```
{
"axis_max" : 470.0,
"axis_min" : 360.0,
"bounding_polygon":
[
        [ 400.0,  400.0, 0.0 ],
        [-300.0,  400.0, 0.0 ],
        [-300.0, -300.0, 0.0 ],
        [ 400.0, -300.0, 0.0 ]
],
"class_name" : "SelectionPolygonVolume",
"orthogonal_axis" : "Z",
"version_major" : 1,
"version_minor" : 0
}
~   
```

```
axis_max: カメラから対象物の切り出し位置最大（単位mm）
axis_min: カメラから対象物の切り出し位置最小（単位mm）
※カメラからaxis_minの位置からaxis_maxの位置までの点群を切り出してicpの対象とします。

bounding_polygon:　切り出す点群のレンジ（x,y,z）
```

## 起動方法

~~~
cd ~/catkin_ws/src/rovi/MasterTeachMethod
~~~

カメラ(SXGA)

```
./MasterTeachMethodSxga.sh
```

カメラ(VGA)

```
./MasterTeachMethodVga.sh
```

MasterTeachingプロセス

```
./launch_MasterTeachMethod.sh
```