<使い方>
  test.pyに記載していますが、
　argv = ["-A20,250,-100,100,250,350","-m 2"]
  のように、パラメータを「"」で囲い、「,」区切りで定義します。

  関数呼び出しは、以下のように定義したargvと点群データのscene(numpy::array)を引数にします。
  戻り値として、リターンコード、normalize後の点群データの配列(result)です。
  pcはx,y,zの順のNx3の配列です。

　result = yodpy2.normalize(argv,scene)
  retcode = result[0]
  pc = result[1]


<サンプルの起動方法>
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
 
  result = loadPLY("out.ply")
  retcode = result[0]
  scene = result[1]
