#! /usr/bin/python
# -*- coding: utf-8 -*-
import yodpy2
import numpy as np
#import stl as m

#####################################
# call normalize
#####################################
#normalizeパラメータ
#argv = ["-a","-r 10","-d 20", "-s 2.1","-A20,250,-100,100,250,350","-m 2","--notexture","-t 5,6","-o test.ply","out.ply"]

#TEST
argv = ["-A20,250,-100,100,250,350","-m 2","-o test.ply","out.ply"]

### TEST2
#argv = ["-A-50,50,-50,50,-50,50","-m 0.2","-r 2","-d 2","-o out_01_withoutnoise-test.ply","out_01_withnoise.ply"]

### TEST3
#argv = ["-A-40,40,-30,50,-20,60","-m 0.5","-r 3","-d 2","-o out_02_withoutnoise-test.ply","out_02_withnoise.ply"]
#argv = ["-A-40,40,-30,50,-20,60","-m 1.0","-r 3","-d 2","-o out_02_withoutnoise-test.ply","out_02_withnoise.ply"]

#print('argv type=',type(argv))
#print('argv=',argv)

#call normalize
result = yodpy2.normalize(argv)

#結果表示
print('normalize result=',result)
