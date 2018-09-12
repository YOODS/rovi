#! /usr/bin/python
# -*- coding: utf-8 -*-
import yodpy2
import numpy as np
#import stl as m


#####################################
# call loadPLY (テスト用)
# normalizeで渡す点群データのarrayを作る
#####################################
result = yodpy2.loadPLY("out.ply")
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
	#argv = ["-r 10","-d 20", "-s 2.1","-A20,250,-100,100,250,350","-m 2"]

	#TEST
	argv = ["-A20,250,-100,100,250,350","-m 2"]

	### TEST2
	#argv = ["-A-50,50,-50,50,-50,50","-m 0.2","-r 2","-d 2"]

	### TEST3
	#argv = ["-A-40,40,-30,50,-20,60","-m 0.5","-r 3","-d 2"]
	#argv = ["-A-40,40,-30,50,-20,60","-m 1.0","-r 3","-d 2"]

	#print('argv type=',type(argv))
	#print('argv=',argv)

	#call normalize
	result = yodpy2.normalize(argv,scene)
	retcode = result[0]
	pc = result[1]

	#結果表示
	print('normalize retcode=',retcode)
	#print('nomalize pc  type=',type(pc))
	#print('normalize pc=',pc)
