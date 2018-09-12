<使い方>
  test.pyに記載していますが、
　argv = ["-A20,250,-100,100,250,350","-m 2","input.ply"]
  のように、パラメータを「"」で囲い、「,」区切りで定義します。

  関数呼び出しは、以下のように定義したargを引数にし、リターンコード、normalize後の点群データがresultに返ります。
  pcはx,y,zの順のNx3の配列です。

　result = yodpy2.normalize(argv)
  retcode = result[0]
  pc = result[1]


<サンプルの起動方法>
  ・python2.7
    python2 test.py

    /usr/bin/pythonのシンボリックリンク元python2.7なら以下でも良い
    python test.py

  ・python3
    python3 test.py
