# Robot Calibration

## Definition of symbols
1. Coordinate transformation matrix s
<img src="https://latex.codecogs.com/gif.latex?{}^{A}T_{B}" />
hows the transformation matrix of the coordinate system B viewed from the coordinate system A.

2. Coordinate system symbol

|Symbol | Coordinate system|
|:----|:----|
|b|robot base|
|m|robot end|
|c|camera|
|s|Object (rigid body)|

## Calibration principle (hand eye camera)
For the hand eye the following conversion is established.

  <img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{s}={}^{b}T_{m}\cdot{}^{m}T_{c}\cdot{}^{c}T_{s}~~~~-(1)" />

Solver of visp_hand2eye

<img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{s}">is constant,

input <img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{m}"> and <img src="https://latex.codecogs.com/gif.latex?{}^{c}T_{s}"> as constant values, and solve unknown <img src="https://latex.codecogs.com/gif.latex?{}^{m}T_{c}">.

## Calibration principle (fixed camera)
For fixed cameras, the conversion that we want to solve is <img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{c}" />. Therefore, substitute <img src="https://latex.codecogs.com/gif.latex?{}^{m}T_{c}" /> of the above equation to obtain the following equation.

  <img src="https://latex.codecogs.com/gif.latex?{}^{m}T_{s}={}^{m}T_{b}\cdot{}^{b}T_{c}\cdot{}^{c}T_{s}~~~~-(2)" />

From this equation, by using <img src="https://latex.codecogs.com/gif.latex?{}^{m}T_{b}(={}^{b}T_{m}^{-1})"> as input instead of <img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{m}">, we can solve in the same solver.

## Evaluation of calibration result
You can get the calibration result with a file in CSV format.

1. Output file 

~~~
$HOME/.ros/
~~~

2. input.txt

The file save the input information (<img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{m}">,<img src="https://latex.codecogs.com/gif.latex?{}^{c}T_{s}">)

3. result.txt

Save the verification result (<img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{s}">,<img src="https://latex.codecogs.com/gif.latex?{}^{m}T_{s}">) corresponding to each line of input.

These are inherently invariant Transforms, but vary due to various errors.

## Install

1. Install VISP

~~~
https://visp.inria.fr/
~~~

~~~
sudo apt-get install ros-kinetic-visp
sudo apt-get install ros-kinetic-visp-hand2eye-calibration
~~~

## Run

After starting RoVI, start adding the following.

1. Startup

~~~
roslaunch calib.launch
~~~

2. Tuning

The software recognizes the calibrated board in real time and outputs its rigid body coordinates to gridboard/tf topic.

Check the gridboard/image_out topic to see if the calibrated board is recognized correctly. Correct recognition has undergone the following steps.

The figure below is an example when recognition is correct

<img src="fig2.png">  

On the other hand, if you can not recognize it, it will be as shown below.

<img src="fig1.png">  

This is when the binarization threshold is not appropriate. The threshold of binarization is set by the bin_param 0 parameter as follows.

~~~
rosparam set /gridboard/bin_param0 50
~~~

## Topics

1. To subscribe

|Topic name|Type|Description|
|:----|:----|:----|
|/rovi/left/image_rect|Image|Rectify image of the reference camera (left)|
|/robot/tf|Transform|Base coordinate reference robot machine end coordinate image|
|/robot/euler|Transform|Base coordinate reference robot machine end coordinate (Euler angle). The data published on this topic will be converted to Quaternion and reissued to /robot/carte. It is prepared as an interface for manual input.|
|/solver/X0|Bool|clear the acquired data|
|/solver/X1|Bool|Store data (Transform pair of object and robot) in buffer. Notify completion of processing at /solver/X1|
|/solver/X2|Bool|Calculate the coordinate transformation from the machine end to the camera from the stored data and output it to the parameter /robot/calib/mTc (bTc for fixed camera). Notify completion of computation at /solver/Y2|

2. To publish

|Topic name|Type|Description|
|:----|:----|:----|
|/gridboard/image_out|Image|Recognition result of calibrated board|
|/gridboard/tf|Transform|Transformation of coordinates with respect to the camera coordinates of the calibrated plate estimated from the camera image|
|/solver/Y1|Bool|It is asserted at the completion of X1 processing|
|/solver/Y2|Bool|It is asserted at the completion of X2 processing|

## Operation

1. Input Data  

Normally, coordinate values ​​are obtained from the robot via communication and published to /robot/tf. If communication can not be made, it is issued to /robot/euler by a tool such as **rqt**. /robot/euler converts Euler angle notation coordinate transformation input into Quaternion and reissues it immediately /robot/tf.

2. コマンド  
You should trigger /solver/X0~X2. Refer to the explanation of Topic for the function of each contact. The basic operation is as follows.

  - Clear coordinate buffer  
  assert /solver/X0
  - Store coordinate (robot+calibration board)  
  Move the robot while capturing the calibrated plate in the camera field of view, assert /solver/X1. This will store the current coordinates in the buffer. If robot coordinates are input, assert /solver/X1, then assert /robot/tf or /robot/euler.  

  - Calcurate calibration  
  By asserting /solver/X2 we calculate the coordinate transformation to the camera. /Solver/Y2 is asserted when operation is completed. The result is written to the parameter /robot/calib /mTc (hand eye), /robot/calib/bTc (fixed camera).

3. Evaluation of calibration result  
Let's evaluate the results from input.txt and result.txt in $HOME/.ros
4. Save images  
Please use imsave.js. r-calib directory it under this directory. Saving is a raw image with left and right combined. Modify it to an appropriate file name and use it.

~~~
./imsave 1
~~~

## パラメータ一覧

- "n_circles_x"	Number of markers in the X axis direction (default value = 13)
- "n_circles_y"	Number of markers in the Y axis direction (default value = 19)
- "unitleng"		Distance between marker center of gravity (default value = 60.0) _Change to the actual calibrated board dimension_
- "distance_between_circles"	The ratio of the maximum distance from the marker end to the next marker end to the marker diameter (default value = 1.2)

- "n_circles_minimum"	The minimum number of markers to be included in the camera image (default value = 9)
- "min_rate"	The minimum allowable size of the size of the entire caliber plate (ratio to image size, default value = 0.25) 
- "max_rate" Maximum allowable size of the entire caliber plate (ratio to image size, default value = 2.00)
- "fitscore"	When forcedly elliptically approximating the extracted contour line, if the extent of observation point is on the ellipse, whether it is judged that the contour line == ellipse (default value = 0.95)
- "fitrange"	Distance between observation point and ellipse to judge that it is on the above (tolerance, default value = 1.50)

- "do_qualize_hist"	Do histogram equalization (1) or not (0)? (Default value = 0)
- "do_smoothing"	Do smoothing (1) or not (0)? (Default value = 1)
- "bin_type"	Binarization type (0: Normal binarization, 1: discriminant analysis binarization, 2: adaptive binarization) (default value = 0) (In the case of 2, it is not very useful because it is not very good.
- "bin_param0"	When bin_type == 0, the threshold value. 2 block size in case.
- "bin_param1"	When bin_type == 2, the offset value from the average value.
