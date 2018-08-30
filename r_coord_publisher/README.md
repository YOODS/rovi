# ロボット座標パブリッシャー

TCP 7373番で
(X,Y,Z,A,B,C)(F1,F2)\n
形式の文字列を待ち、
Transform型で/robot/eulerにPublishするノード。

## 起動方法
~~~
roslaunch rovi r_coord_publisher.launch
~~~

## 動作確認
~~~
rostopic echo /robot/euler
~~~
した状態で、
~~~
./test_r_coord_publisher.js
~~~
で動作確認できる。

(./test_r_coord_publisher.js  
の代わりに  
telnet localhost 7373  
で直接  
(1.1,2.2,3.3,4,5,6.01)(0,7)  
など打ち込んでも動作確認可能。)

## 注意点
1. クライアントからは1行ごとにflushされる前提。  
(そうしないとバッファ処理など面倒なことになりそう。)
2. flushされても、あまりに速く続けて2行来るとPublishが追いつかない  
(ROSがそういうものらしい)  
ので、flush間は少し時間を開けること。  
(take-w手元のPCでは、1msec間隔だとPublishが追いつかないことがあった。  
0msec間隔だとまったく追いつかない。  
2msec間隔なら大丈夫な模様。  
現実的にはそんなに速くクライアントから続けて来ないはずではあるが。)
