#!/bin/bash
rosparam set /param/P1 60.0
rosparam set /param/P2 5.0
rosparam set /param/P3 2.42106055e+03
rosparam set /param/P4 0.0
rosparam set /param/P5 640
rosparam set /param/P6 0.0
rosparam set /param/P7 2.42106055e+03
rosparam set /param/P8 512
rosparam set /param/P9 0.0
rosparam set /param/P10 1.0

./param_perf.js &
