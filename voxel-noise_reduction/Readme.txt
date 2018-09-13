<使い方>
  1.mesh初期化
    パラメータ:
	area:省略不可
		area=((xmin,xmax),(ymin,ymax),(zmin,zmax))
	mesh:省略可能
		mesh=0.2
    戻り値:
	retcode:処理結果
		  0 正常終了
		-10 areaパラメータ設定エラー
    使用例:
    	retcode = yod2.makeMesh(["area=((-50,50),(-50,50),(-50,50))","mesh=0.2"])

  2.normalize
    パラメータ:
	scene:点群データ配列(x,y,zの順のNx3の配列)
    戻り値:
	result:配列
		0番目:処理結果
		  	  0 正常終了
			-20 sceneエラー
			-90 その他エラー

    使用例:
	result = yodpy2.normalize(scene)
	retcode = result[0]
	pc = result[1]

<サンプルの実行方法>
  ・python2.7
    python2 test.py

    /usr/bin/pythonのシンボリックリンク元python2.7なら以下でも良い
    python test.py

  ・python3
    python3 test.py

<その他>
　test.pyでは、PLYファイルから点群データのnumpy::arrayを作る関数も使っています。
　※テスト用にnormalizeの入力に使うため。
  戻り値はリターンコードとPLYファイルから作成した点群データのnumpy::arrayの配列です。
 
  result = loadPLY("input.ply")
  retcode = result[0]
  scene = result[1]
