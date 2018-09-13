# 点群データフィルタ

点群データの以下の後処理を行う、Pythonライブラリ

1. normalize

VoxelGridし過剰な点群を削除します。

2. ???


## 準備
1. Pybind11のインストール

~~~
sudo -H pip install pybind11
~~~
もしpip(Pythonのパッケージインストーラ)がインストールされてなければ
~~~
sudo apt install python-pip python-dev
~~~
パッケージ一覧を
~~~
pip list
~~~
で見て確認しよう

2. yodpy2.soのビルド

yodpy2.soがYOODSの点群ライブラリ。ビルドはこのディレクトリで
~~~
make
~~~

3. .soをPYTHONPATHにコピー
pythonは環境変数PYTHONPATHにあるパッケージをimport文にて検索するので、yodpy2.soをそこにコピーする。普通は~/catkin_ws/devel/lib/python2.7/dist-packages/以下です。
~~~
cp yodpy2.so ~/catkin_ws/devel/lib/python2.7/dist-packages/
~~~

## サンプルコード


