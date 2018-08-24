# ROSの言語選択ガイドライン

実装をうまく行うには言語の適材適所化が重要です。
ROSのTutrialにはC++とPythonしか登場しませんが、実は開発環境は以下の4言語をサポートしています。

1. C++
2. Python
3. Javascript(Nodejs)
4. Lisp

## 言語の概要


### C++  
説明省略  
下記２言語ではムリ(遅い)なときの最後の選択と思いましょう。

### Python  
近年科学技術計算でFortranを葬り去った言語。Pythonは言語自体が優れているというよりは、
1. OpenCVやDNNなど、優れたOSSライブラリ(.so)にはPythonラッパーが組み込まれている
2. 各ライブラリ固有のデータ構造がPythonのArray(numpy)にて統一されている
ため、ライブラリの利用が劇的に簡単+高速(.soなので当然)であることが最大のメリット。  

逆にPythonの内部処理(for..など)は遅いので、Pythonのプログラミングはライブラリの機能から考えることが大事。

### Javascript(Nodejs)  
Javascriptは元々WebBrowserの組み込み言語でしたが、そのエンジン部をバックエンドに用いたものがNodejsです。近年通信系のシステムではCやJavaを葬り去ったと言えます。  
Javascriptは純粋な言語ではなく、処理系にイベントループが内包されていることが特徴です。大半のfunctionは結果がコールバックで返され、呼び出し側はブロックされません。この特徴がサーバサイドのトランザクション処理に向いていることがNodejsの広まった要因です。  
ROSでもこの構造を上手く利用し、NodejsのイベントループにROSのイベント(Subscribe)を組み込んでいます(ros::spinが無い)。つまりNodejsで通信等の処理を行いつつ、ROSのイベントを同時に処理をすることができます。これはPythonでは実装不可能、C++でもマルチスレッドにしないと出来ない処理です。  
以下はまりポイントにも注意してください
1. Javascriptでは、すべての処理(function)は何らかのイベントを起点にして始まっています。このためイベントハンドラ内でforループとかを作ってしまうとJavascriptのイベントループが停止します。  
2. 逆にfunction呼び出しをブロックさせたいときの記述が難しかったのですが、近年async..awaitの記法が定着し大変見やすくなりました。ただしブロック(await)させたいfunctionの内部設計はPromiseというclassを理解してないと出来ないので、こういうときは上級者に見てもらいましょう。

ちなみにROSにNodejsを導入したRethinkRobotics社はDr.Rodney Brookの創業した会社です(iRobot社の方が有名ですが)

### Lisp
誰か使ったことないでしょうか？

## ROSでの機能分担

|言語|用途|ライブラリ|RoVIファイル例|
|:----|:----|:----|:----|
|Python|画像処理  Transform(座標変換)演算|OpenCV,numpy,scipyなど|...solver.py|
|Nodejs|通信処理  シーケンス処理  Browserバックエンド|Net,Eventなど|...socket.js|
|C++|高速処理| | |
