#! /usr/bin/python
# -*- coding: utf-8 -*-
import yodpy2
import numpy as np
#import stl as m


#####################################
# call makeMesh
# meshを作る
#####################################
# area=は省略不可,mesh=を省略した場合は0.1が設定されたものとみなす。

# NG test area未設定
#retcode = yodpy2.makeMesh(mesh=2.0)

# OK test mesh省略
#retcode = yodpy2.makeMesh(area=((20,250),(-100,100),(250,350)))

# OK test area,mesh設定
retcode = yodpy2.makeMesh(area=((20,250),(-100,100),(250,350)),mesh=2.0)
#retcode = yodpy2.makeMesh(area=((-50,50),(-50,50),(-50,50)),mesh=0.2)
#retcode = yodpy2.makeMesh(area=((-40,40),(-30,50),(-20,60)),mesh=1.0)

print('makeMesh retcode=',retcode)


#####################################
# call loadPLY (テスト用)
# normalizeで渡す点群データのarrayを作る
#####################################
if retcode == 0:
	result = yodpy2.loadPLY("out.ply")
	#result = yodpy2.loadPLY("out_01_withnoise.ply")
	#result = yodpy2.loadPLY("out_02_withnoise.ply")
	retcode = result[0]
	scene = result[1]
	print('loadPLY retcode=',retcode)

#####################################
# call normalize
#####################################
if retcode == 0:
	#normalizeパラメータ
	# PLYファイルに書き出さないので、-a --notexture -c -o -t は使わなくなった
	# PLYファイルを読み込まないので、plyファイル名の指定も不要になった
	# -r(noise judge(voxel)) -d(r=noise judge(pixel)) -s(smooth)も使わない
	#
	# 一応、-r -d -s の説明を書いておく
	# r=noise judge(voxel)
	# 注目しているvoxelの近傍をぐるっとさがして、隣接点があるかないかでノイズか否かを判定しています。
	#
	# d=noise judge(directions)
	# 近傍Voxel探索でノイズと判定する場合のカウンタ閾値です
	#
	# s=smooth
	# 出力時に平滑化するかどうか・・・指定値は注目点の周囲半径です。
	#
	#いずれも今回の目的の間引きには不要です。
	#
	#argv = ["-r 10","-d 20", "-s 2.1","-A20,250,-100,100,250,350","-m 2"]
	#
	# -r -dは何か設定した方が良さげ？なので、元のソースのデフォルト値をそのまま使っている。

	#call normalize
	result = yodpy2.normalize(scene)
	retcode = result[0]
	pc = result[1]

	#結果表示
	print('normalize retcode=',retcode)
	#print('nomalize pc  type=',type(pc))
	#print('normalize pc=',pc)


#####################################
# call normalize 2回目
#####################################
if retcode == 0:
	result = yodpy2.normalize(scene)
	retcode = result[0]
	pc = result[1]

	#結果表示
	print('normalize retcode=',retcode)
	#print('nomalize pc  type=',type(pc))
	#print('normalize pc=',pc)


#####################################
# 別のPLYファイルで処理してみる
#####################################
# call makeMesh
# meshを作る
#####################################
# area=は省略不可,mesh=を省略した場合は0.1が設定されたものとみなす。
if retcode == 0:
	retcode = yodpy2.makeMesh(area=((-50,50),(-50,50),(-50,50)),mesh=0.2)

	print('makeMesh retcode=',retcode)


#####################################
# call loadPLY (テスト用)
# normalizeで渡す点群データのarrayを作る
#####################################
if retcode == 0:
	result = yodpy2.loadPLY("out_01_withnoise.ply")
	retcode = result[0]
	scene = result[1]
	print('loadPLY retcode=',retcode)


#####################################
# call normalize 3回目
#####################################
if retcode == 0:
	result = yodpy2.normalize(scene)
	retcode = result[0]
	pc = result[1]

	#結果表示
	print('normalize retcode=',retcode)
	#print('nomalize pc  type=',type(pc))
	#print('normalize pc=',pc)
