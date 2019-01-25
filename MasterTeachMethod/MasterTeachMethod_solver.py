#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
import roslib
import rospy

import time

#デバッグファイル名作成用
import datetime

#readPLY用
import open3d as o3d

#マスターRTをオブジェクトとして保存する
import pickle
import os.path
import shutil

#マスターとの差分を求める
from pyquaternion import Quaternion

#川村さんロジック
import icp
from icp import ICPEstimator
import crop

from rovi.msg import Floats
from rovi.msg import PickingPose
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import Trigger,TriggerRequest
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point
from geometry_msgs.msg import Transform
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../script'))
import tflib
import tf
from jsk_rviz_plugins.msg import OverlayText

#マスター点群（クロップ済みのロボット座標点群）ファイル名
master_ply_file_name = 'master.ply'

#マスター位置(RT)ファイル名 ※bTm
master_rt_file_name = 'master_bTm_rt'

#オリジナルの点群ファイル
proc_ply = "/tmp/test.ply"

#ロボット座標の点群ファイル(proc_plyをクロップしてロボット座標にした点群ファイル)
scene_ply = "/tmp/MasterTeachMethod_scene.ply"

def xyz2quat(e):
  tf=Transform()
  k = math.pi / 180 * 0.5
  cx = math.cos(e.rotation.x * k)
  cy = math.cos(e.rotation.y * k)
  cz = math.cos(e.rotation.z * k)
  sx = math.sin(e.rotation.x * k)
  sy = math.sin(e.rotation.y * k)
  sz = math.sin(e.rotation.z * k)
  tf.translation.x=e.translation.x
  tf.translation.y=e.translation.y
  tf.translation.z=e.translation.z
  tf.rotation.x = cy * cz * sx - cx * sy * sz
  tf.rotation.y = cy * sx * sz + cx * cz * sy
  tf.rotation.z = cx * cy * sz - cz * sx * sy
  tf.rotation.w = sx * sy * sz + cx * cy * cz
  return tf

def check_obst(e):
  global scnPn
  q=xyz2quat(e)
#  print 'base2work quat',e
  i=np.linalg.inv(tflib.toRT(q))
#  print 'base2work inv rt',i
  P=np.vstack((scnPn.T,np.ones((1,len(scnPn)))))
  tp=np.dot(i[:3],P).T
#  print "work2base points",tp.shape,tp
  W=np.where(tp.T[0]>=-6.0)
  tp=tp[W[len(W)-1]]
  W=np.where(tp.T[0]<=+6.0)
  tp=tp[W[len(W)-1]]
  W=np.where(tp.T[1]>=-6.0)
  tp=tp[W[len(W)-1]]
  W=np.where(tp.T[1]<=+6.0)
  tp=tp[W[len(W)-1]]
  W=np.where(tp.T[2]>=+8.0)
  tp=tp[W[len(W)-1]]
  W=np.where(tp.T[2]<=+50.0)
  tp=tp[W[len(W)-1]]
#  d=tp.astype(np.float32)
#  cv2.ppf_match_3d.writePLY(d,'obs.ply')
  print "tp",len(tp),tp.shape
  return True if (len(tp)<100) else False

def robot_rxyzabc_to_rt(rx, ry, rz, a_rad, b_rad, c_rad):
  matrix44 = np.zeros((4, 4))
  x_mat = np.zeros((4, 4))
  y_mat = np.zeros((4, 4))
  z_mat = np.zeros((4, 4))

  matrix44[3, 3] = 1
  matrix44[0, 3] = rx
  matrix44[1, 3] = ry
  matrix44[2, 3] = rz

  x_mat[0, 0] = 1.0
  x_mat[1, 1] = np.cos(a_rad)
  x_mat[1, 2] = -np.sin(a_rad)
  x_mat[2, 1] = np.sin(a_rad)
  x_mat[2, 2] = np.cos(a_rad)

  y_mat[0, 0] = np.cos(a_rad)
  y_mat[0, 2] = np.sin(b_rad)
  y_mat[1, 1] = 1.0
  y_mat[2, 0] = -np.sin(b_rad)
  y_mat[2, 2] = np.cos(b_rad)

  z_mat[0, 0] = np.cos(c_rad)
  z_mat[0, 1] = -np.sin(c_rad)
  z_mat[1, 0] = np.sin(c_rad)

  z_mat[1, 1] = np.cos(c_rad)
  z_mat[2, 2] = 1.0

  temp_mat1 = np.zeros((3, 3))
  temp_mat2 = np.zeros((3, 3))

  for m in range(3):
    for n in range(3):
      for k in range(3):
        temp_mat1[m, n] += y_mat[m, k] * z_mat[k, n]

  for m in range(3):
    for n in range(3):
      for k in range(3):
        temp_mat2[m, n] += x_mat[m, k] * temp_mat1[k, n]

  for m in range(3):
    for n in range(3):
      matrix44[m, n] = temp_mat2[m, n]

  return matrix44

def P0():
  return np.array([]).reshape((-1,3))

def np2Fm(d):  #numpy to Floats (unit is meter for RViZ)
  f=Floats()
  f.data=np.ravel(d) / 1000.0
  return f

def np2FmNoDivide(d):  #numpy to Floats (unit is already meter for RViZ)
  f=Floats()
  f.data=np.ravel(d)
  return f

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def pc2arr(pc):
  return np.asarray(pc.points)

#デバッグ用のデータを保存する（点群とbTmファイル）
def copyData():
  global bTmCurrent, debug_data_dir

  dateStr=datetime.datetime.today().strftime("%Y_%m_%d_%H%M%S")

  if(not os.path.isdir(debug_data_dir)):
    os.mkdir(debug_dir)

  #ply
  if(os.path.exists(proc_ply)): #カメラ座標点群
    shutil.copy(proc_ply,debug_data_dir + "/" + dateStr + "_ps.ply")  
  if(os.path.exists(scene_ply)): #ベース座標点群 (クロップ後の点群)
    shutil.copy(scene_ply,debug_data_dir + "/" + dateStr + "_mTb_crop.ply")  

  #bTm
  print "write Current RT(bTm) file."
  try:
    rt_file=debug_data_dir + "/" + dateStr + "_mTb_rt"
    print 'rt_file:',rt_file
    with open(rt_file, 'wb') as f:
      pickle.dump(bTmCurrent, f)
  except Exception as e:
    print "Current RT file write exception: ", e

  return

def readPLY(path):
  if( not os.path.exists(path)): return P0()
  #if not fileExists(path): return P0()
  pc=o3d.PointCloud()
  pc=o3d.read_point_cloud(path)
  return pc2arr(pc)

def clearModel():
  global modPn
  modPn=P0()
  return

def clearScene():
  global scnPn
  scnPn=P0()
  return

def publishModel():
  global modPn
  pub_mdf.publish(np2F(modPn))
  return

def publishScene():
  global scnPn
  pub_scf.publish(np2F(scnPn))
  return

#川村さんロジック呼び出し 
def call_estimator():
  global master_ply

  RT = None 
  #kawamuraさんロジック 
  RT =  icp.EstimateRT(
    estimator,					#estimatorオブジェクト
    master_ply,					#マスター点群データ
    scene_ply)					#フランジ座標点群データ 
  return RT

def cb_ps(msg): #callback of ps_floats
  print "cb_ps called!"

  global bTmLat, mTc, scnPn, modPn, bTm, crop_file_path, bTmCurrent, master_dir, master_ply

  clearScene()
  P=np.reshape(msg.data,(-1,3))
#  cnPn=np.vstack((scnPn,P))
#  clearScene()

  # crop 2019.01.23
  # icpではクロップ済みのロボット座標点群ファイルを使うので、
  # ここでオリジナルの点群をクロップしておく
  #P=crop.crop(crop_file_path,P,verbose=True)
  P=crop.crop(crop_file_path,P,verbose=False)

  # 2019.01.23 ベース座標の点群にする
  n,m=P.shape

  #print "PointCloud In:",n
  #print "PC(camera in mm)",P

  P=np.vstack((P.T,np.ones((1,n))))

  # 2019.01.24 ターゲットは現在位置のbTmを使う
  # マスター作成時はbTmCurrent = bTm
  #P=np.dot(bTm,np.dot(mTc,P)).T
  P=np.dot(bTmCurrent,np.dot(mTc,P)).T

  P=P[:,[0,1,2]]
  scnPn=np.vstack((scnPn,P))

  publishScene() 

  cv2.ppf_match_3d.writePLY(scnPn.astype(np.float32), scene_ply)


  ##############################################################
  ## マスターをファイルに書いてみる
  ## 残すのはカメラ座標の点群ファイルと川村さんロジックからのRT
  ##############################################################
  #ファイルが存在しない場合だけ作成する。
  if(not os.path.isdir(master_dir)):
    os.mkdir(master_dir)

  if( not os.path.exists(master_ply)):
    print "make Master PLY file."

    # 2019.01.23 ベース座標の点群にする
    #copy ply file
    if(os.path.exists(scene_ply)):
      shutil.copy(scene_ply,master_ply)  
      #rviz表示用
      p=readPLY(master_ply)
      print 'load PC',p.dtype,p.shape
      modPn=np.vstack((modPn,p))
      publishModel()
    else:
      return

    if( not os.path.exists(master_rt)):
      print "write Master RT file."

      try:
        with open(master_rt, 'wb') as f:
          pickle.dump(bTm, f)
      except Exception as e:
        print "master_rt file write exception: ", e
        pub_tf.publish(tf)
        return
  else:
    print "Master PLY file exists."
  ## マスターファイル作成（ここまで）###############################################################

  print "Call Y1"
  pub_Y1.publish(True)

  print "#####  end cb_ps " +datetime.datetime.today().strftime("%Y_%m_%d_%H%M%S")

  return

def cb_X0(f):
  global scnPn, mfoPn, mnpPn, mpiPn
  print "X0:scene reset"
  scnPn=P0()
  mfoPn=P0()
  mnpPn=P0()
  mpiPn=P0()
  pub_scf.publish(np2Fm(scnPn))
  pub_mfof.publish(np2FmNoDivide(mfoPn))
  pub_mnpf.publish(np2FmNoDivide(mnpPn))
  pub_mpif.publish(np2FmNoDivide(mpiPn))
  return

def cb_X1(f):
  print "##### start cb_X1 " +datetime.datetime.today().strftime("%Y_%m_%d_%H%M%S")

  cb_X0(True)
  global bTm,bTmLat
  bTmLat=np.copy(bTm)
  print "bTm latched",bTmLat
  genpc=None
  try:
    genpc=rospy.ServiceProxy('/rovi/pshift_genpc',Trigger)
    req=TriggerRequest()
    ret=genpc(req)      #will continue to callback cb_ps
    print "genpc result: ",ret
    if (ret.success==False):
      pub_Y1.publish(False)
  except rospy.ServiceException, e:
    print 'Genpc proxy failed:',e
    pub_Y1.publish(False)

  return

def cb_X2(f):
  print "cb_X2 start"
  print "##### start cb_X2 " +datetime.datetime.today().strftime("%Y_%m_%d_%H%M%S")

  stt=time.time()        ### SW

  #ロボット座標計算用変数も使う
  global  mfoPn, mnpPn, mpiPn, bTmLat, mTc, estimator, is_debug, bTm

  # Picking Pose to be returned
  pp = PickingPose()
  pp.ok = False
  pp.errorreason = ""
  pp.x = 0
  pp.y = 0
  pp.z = 0
  pp.a = 0
  pp.b = 0
  pp.c = 0
  # 2019.01.24 end

  #----------------------------------------------
  #kawamuraさんロジック 
  #execute estimator
  RT11 = call_estimator()

  print '_call_estimator done.           elapsed={0}'.format(time.time()-stt)+'[sec]'  ### SW

  ### デバッグ用のファイルを保存
  if(is_debug):
    copyData()

  #RT11がNoneの場合はエラー(川村さんロジックでexceptionなどが発生するとこの状態でRTが返却される)
  if RT11 is None:
    print "ERROR: X2 estimator() failed. RT is None."
    pub_Y2.publish(pp)
    return

  #RT11のサイズが0の場合はエラー
  if (RT11.size == 0):
    print "ERROR: X2 estimator() failed. RT size is zero."
    pub_Y2.publish(pp)
    return

  # ロボットへの応答
  if(robot_type == 'Mitsubishi'):	# Mitsubishi
    # 2019.01.23 ベース座標でicpしているのでRT11がそのままつかえるはず
    tfeuler=tflib.fromRTtoEulerCBA(RT11)
    print 'RTtoEulerCBA(org): %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f' % (tfeuler[0],tfeuler[1],tfeuler[2],tfeuler[3],tfeuler[4],tfeuler[5],tfeuler[6])
    tfeuler=tflib.fromRTtoEulerCBA(np.linalg.inv(RT11))
    print 'RTtoEulerCBA(inv org): %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f' % (tfeuler[0],tfeuler[1],tfeuler[2],tfeuler[3],tfeuler[4],tfeuler[5],tfeuler[6])
  elif(robot_type == 'Funac'):		# Funac
    # 2019.01.23 ベース座標でicpしているのでRT11がそのままつかえるはず
    tfeuler=tflib.fromRTtoEulerABC(RT11)
    print 'RTtoEulerABC(org): %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f' % (tfeuler[0],tfeuler[1],tfeuler[2],tfeuler[3],tfeuler[4],tfeuler[5],tfeuler[6])
    tfeuler=tflib.fromRTtoEulerABC(np.linalg.inv(RT11))
    print 'RTtoEulerABC(inv org): %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f' % (tfeuler[0],tfeuler[1],tfeuler[2],tfeuler[3],tfeuler[4],tfeuler[5],tfeuler[6])


  #ロボットに通知する位置情報を設定
  pp.ok = True
  pp.errorreason = ""
  pp.x = tfeuler[0]
  pp.y = tfeuler[1]
  pp.z = tfeuler[2]
  pp.a = tfeuler[3]
  pp.b = tfeuler[4]
  pp.c = tfeuler[5]

  if(robot_type == 'Mitsubishi'):	# Mitsubishi
    #ここで、角度の変換が必要かも・・
    #ロボットは 0 <-- 180,180 --> 0 で、
    #川村さんが、
    # 1.  180 <-- 0 --> 180
    # 2. -180 <-- 0 --> 180
    # で返す場合には、roll,pitch,yawを変換する必要がある。
    # 1.
    #pp.a = 180.0 - pp.a
    #pp.b = 180.0 - pp.b
    #pp.c = 180.0 - pp.c
    # 2.
    #pp.a = -180.0 - pp.a
    #pp.b = -180.0 - pp.b
    #pp.c = -180.0 - pp.c

    (-1 if (pp.a<0) else 1) * 180-(pp.a-(pp.a/360)*360)
    (-1 if (pp.b<0) else 1) * 180-(pp.b-(pp.b/360)*360)
    (-1 if (pp.c<0) else 1) * 180-(pp.c-(pp.c/360)*360)

  print "******** Picking Pose(after): x=", pp.x, "y=", pp.y, "z=", pp.z, "a=", pp.a, "b=", pp.b, "c=", pp.c

  print "Call Y2"
  pub_Y2.publish(pp)

  print '_cb_X2 done.           elapsed={0}'.format(time.time()-stt)+'[sec]'  ### SW
  print "##### end cb_X2 " +datetime.datetime.today().strftime("%Y_%m_%d_%H%M%S")

  return

def cb_tf(tf):
  global bTm, save_master_bTm, bTc, cTs, bTmCurrent
  
  bTmCurrent=tflib.toRT(tf)

  ###################################
  ## マスターファイルを読み込む
  ###################################
  if(save_master_bTm==False):
    print "cb_tf:save_master_bTm"
    bTm=bTmCurrent
    save_master_bTm = True

    #マスターRTファイルがあれば、bTmをマスターRTファイルのものに入れ替える
    if(os.path.exists(master_rt)):
      print "cb_tf:read master_bTm_rt"
      try:
        with open(master_rt, 'rb') as f:
          bTm = pickle.load(f)
      except Exception as e:
        print "master_btm_rt file read exception: ", e

  print 'cb_tf bTm:\n', bTm
  print 'cb_tf bTmCurrent:\n', bTmCurrent

  #以下のpublishはタイミングを合わせるためだけに呼び出している
  #jsで何かしらの処理を行ったりはしない
  mTs=np.dot(np.dot(bTmCurrent.I,bTc),cTs)
  pb_mTs.publish(tflib.fromRT(mTs))

  return

# オフラインテスト用
def cb_tf2(tf):
  global bTm, save_master_bTm, bTmCurrent

  #bTm=tflib.toRT(xyz2quat(tf))

  bTmCurrent=tflib.toRT(xyz2quat(tf))

  ###################################
  ## マスターファイルを読み込む
  ###################################
  if(save_master_bTm==False):
    print "cb_tf2:save_master_bTm"
    bTm=bTmCurrent
    save_master_bTm = True

    #マスターRTファイルがあれば、bTmをマスターRTファイルのものに入れ替える
    if(os.path.exists(master_rt)):
      print "cb_tf2:read master_bTm_rt"
      try:
        with open(master_rt, 'rb') as f:
          bTm = pickle.load(f)
      except Exception as e:
        print "master_btm_rt file read exception: ", e

  print "cb_tf2 bTm ",bTm
  print 'cb_tf2 bTmCurrent:\n', bTmCurrent

  return

########################################################

rospy.init_node("solver",anonymous=True)

###Input topics
rospy.Subscriber("/robot/tf",Transform,cb_tf)				#ロボットからロボット座標を受け取る
rospy.Subscriber("/robot/euler",Transform,cb_tf2) 			#ロボット座標を受け取る（オフライン用） 
rospy.Subscriber("/rovi/ps_floats",numpy_msg(Floats),cb_ps)		#phase shift処理コールバック
#rospy.Subscriber("/solver/X0",Bool,cb_X0)				#Clear scene
rospy.Subscriber("/solver/X1",Bool,cb_X1)				#Capture and genpc into scene
rospy.Subscriber("/solver/X2",Bool,cb_X2)				#Recognize work and calc picking pose


###Params
is_debug = int(rospy.get_param('/solver/debug'))			#デバッグ用ファイル保存有無フラグ
is_verbose = int(rospy.get_param('/solver/verbose'))			#icpでのデバッグウィンドウ表示フラグ
crop_file_path = rospy.get_param('/solver/crop_file_path')		#クロップファイルパス
master_dir = rospy.get_param('/solver/master_dir')			#マスターデータ格納ディレクトリ
debug_data_dir = rospy.get_param('/solver/debug_data_dir')		#デバッグ用ファイル格納ディレクトリ
robot_type = rospy.get_param('/solver/robot_type')			#ロボット種別 Mitsubishi または Funac の文字列
master_ply = master_dir + master_ply_file_name 				#マスター点群（クロップ済みのロボット座標点群）ファイルパス
master_rt = master_dir + master_rt_file_name				#マスター位置(RT)ファイルパス(bTmを保存している)
fitness_threshold = float(rospy.get_param('/solver/fitness_threshold'))	#一致率閾値
xmin = float(rospy.get_param('/volume_of_interest/xmin'))
xmax = float(rospy.get_param('/volume_of_interest/xmax'))
ymin = float(rospy.get_param('/volume_of_interest/ymin'))
ymax = float(rospy.get_param('/volume_of_interest/ymax'))
zmin = float(rospy.get_param('/volume_of_interest/zmin'))
zmax = float(rospy.get_param('/volume_of_interest/zmax'))
sparse_threshold = int(rospy.get_param('/dense_sparse/threshold'))

print "xmin=", xmin, "xmax=", xmax, "ymin=", ymin, "ymax=", ymax, "zmin=", zmin, "zmax=", zmax
print "sparse_threshold=", sparse_threshold
'''
print("is_debug:",is_debug)
print("is_verbose:",is_verbose)
print("crop_file_path:",crop_file_path)
print("diameter:",diameter)
print "robot_type:",robot_type
'''
print "fitness_threshold:",fitness_threshold
print "master_ply:",master_ply
print "master_rt:",master_rt
print "debug_data_dir:",debug_data_dir

###Output topics
pub_scf=rospy.Publisher("/scene/floats",numpy_msg(Floats),queue_size=1)
pub_mdf=rospy.Publisher("/model/floats",numpy_msg(Floats),queue_size=1)
pub_mfof=rospy.Publisher("/model/found/floats",numpy_msg(Floats),queue_size=1)
pub_mnpf=rospy.Publisher("/model/cannotpick/floats",numpy_msg(Floats),queue_size=1)
pub_mpif=rospy.Publisher("/model/picking/floats",numpy_msg(Floats),queue_size=1)
pub_Y1=rospy.Publisher('/solver/Y1',Bool,queue_size=1)    #X1 done
pub_Y2=rospy.Publisher('/solver/Y2',PickingPose,queue_size=1)    #X2 done
pub_msg=rospy.Publisher('/solver/message',OverlayText,queue_size=1)
pb_bTs=rospy.Publisher('/solver/bTs',Transform,queue_size=1)
pb_mTs=rospy.Publisher('/solver/mTs',Transform,queue_size=1)

###Transform
bTmLat=np.eye(4).astype(float)  #robot RT when captured
bTm=np.eye(4).astype(float) 
cTs=np.eye(4,dtype=float)
bTc=np.eye(4,dtype=float)
# 2019.01.24
bTmCurrent=np.eye(4).astype(float)  #robot RT when captured

###Robot Calibration Result
mTc=tflib.toRT(tflib.dict2tf(rospy.get_param('/robot/calib/mTc')))  # arM tip To Camera
#print "mTc=", mTc

###Globals
scnPn=P0()  # Scene points
modPn=P0()  # Model points
mfoPn=P0()  # Model Found points
mnpPn=P0()  # Model Cannot Pick points
mpiPn=P0()  # Model Picking points
save_master_bTm = False

#----------------------------------------------
#kawamuraさんロジック (コンストラクタ)

#print 'mTc:',mTc

#2019.01.09 mm に戻す
voxel_size = 2
#2019.01.07 mm から m　に変更
#voxel_size = 0.002

estimator = icp.CreateEstimator(
  voxel_size=voxel_size,		#voxel_size(mm) 
  neighbor=6,				#neighbor(ノイズと判断する個数)
  fitness_threshold=fitness_threshold,  #一致率閾値
  verbose = is_verbose)                 #show debug window
#----------------------------------------------

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
