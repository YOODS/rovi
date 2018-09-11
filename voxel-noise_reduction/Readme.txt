<使い方>
  test.pyに記載していますが、
　argv = ["-A20,250,-100,100,250,350","-m 2","-o output.ply","input.ply"]
  のように、パラメータを「"」で囲い、「,」区切りで定義します。

  関数呼び出しは、以下のように定義したargを引数にし、リターンコードがresultに返ります。

　result = yodpy2.normalize(argv)

  現状は、入出力ともPLYファイルです。

<サンプルの起動方法>
  python3 test.py
