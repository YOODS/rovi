#!/usr/bin/python

import cv2
import numpy as np
import roslib
import rospy
import yodpy
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

Tolerance=0.001
Rejection=2.5
scene_ply = "/tmp/wrs2018_scene.ply"

def robot_rxyzabc_to_rt(rx, ry, rz, a_rad, b_rad, c_rad):
  matrix44 = np.zeros((4, 4))
  x_mat = np.zeros((4, 4))
  y_mat = np.zeros((4, 4))
  z_mat = np.zeros((4, 4))

  matrix44[3][3] = 1
  matrix44[0][3] = rx;
  matrix44[1][3] = ry;
  matrix44[2][3] = rz;

  x_mat[0][0] = 1.0;
  x_mat[1][1] = np.cos(a_rad)
  x_mat[1][2] = -np.sin(a_rad);
  x_mat[2][1] = np.sin(a_rad);
  x_mat[2][2] = np.cos(a_rad);

  y_mat[0][0] = np.cos(a_rad);
  y_mat[0][2] = np.sin(b_rad);
  y_mat[1][1] = 1.0;
  y_mat[2][0] = -np.sin(b_rad);
  y_mat[2][2] = np.cos(b_rad);

  z_mat[0][0] = np.cos(c_rad);
  z_mat[0][1] = -np.sin(c_rad);
  z_mat[1][0] = np.sin(c_rad);

  z_mat[1][1] = np.cos(c_rad);
  z_mat[2][2] = 1.0;

  temp_mat1 = np.zeros((3, 3))
  temp_mat2 = np.zeros((3, 3))

  for m in range(3):
    for n in range(3):
      for k in range(3):
        temp_mat1[m][n] += y_mat[m][k] * z_mat[k][n]

  for m in range(3):
    for n in range(3):
      for k in range(3):
        temp_mat2[m][n] += x_mat[m][k] * temp_mat1[k][n]

  for m in range(3):
    for n in range(3):
      matrix44[m][n] = temp_mat2[m][n]

  return matrix44

def P0():
  return np.array([]).reshape((-1,3))

def np2Fm(d):  #numpy to Floats (unit is meter for RViZ)
  f=Floats()
  f.data=np.ravel(d) / 1000.0
  return f

def prepare_model(stl_file):
  global model, is_prepared
  result = yodpy.train3D(stl_file, relSamplingDistance = 0.03)
  retcode = result[0]
  model = result[1]
  print('train3D retcode=', retcode)
  print('train3D model size=', len(model))
  print('train3D model=', model)
  if retcode == 0:
    is_prepared = True
  return

def cb_ps(msg): #callback of ps_floats
  print "cb_ps called!"

  if not is_prepared:
    print "ERROR: prepare_model() is NOT done. ignore this ps_floats."
    pub_Y1.publish(False)
    return

  global bTmLat, mTc, scnPn
  P=np.reshape(msg.data,(-1,3))
  n,m=P.shape
  #P=voxel(P)
  print "PointCloud In:",n
  #print "PC(camera in meter)",P
  P=P*1000
  #print "PC(camera in mm)",P
  n,m=P.shape
  #print "PointCloud Out:",n
  P=np.vstack((P.T,np.ones((1,n))))
  P=np.dot(bTm[:3],np.dot(mTc,P)).T
  #print "P2",P
  print P.shape
  if (not np.isnan(xmin)):
    W = np.where(P.T[0] >= xmin)
    P=P[W[len(W)-1]]
  if (not np.isnan(xmax)):
    W = np.where(P.T[0] <= xmax)
    P=P[W[len(W)-1]]
  if (not np.isnan(ymin)):
    W = np.where(P.T[1] >= ymin)
    P=P[W[len(W)-1]]
  if (not np.isnan(ymax)):
    W = np.where(P.T[1] <= ymax)
    P=P[W[len(W)-1]]
  if (not np.isnan(zmin)):
    W = np.where(P.T[2] >= zmin)
    P=P[W[len(W)-1]]
  if (not np.isnan(zmax)):
    W = np.where(P.T[2] <= zmax)
    P=P[W[len(W)-1]]
  #print "where",P
  n,m=P.shape
  print P.shape
  #print "PC",P
  P=P.reshape((-1,3))
  scnPn=np.vstack((scnPn,P))
  pub_scf.publish(np2Fm(scnPn))

  cv2.ppf_match_3d.writePLY(scnPn.astype(np.float32), scene_ply)
  pub_Y1.publish(True)
  return

def cb_X0(f):
  global scnPn
  print "X0:scene reset"
  scnPn=P0()
  pub_scf.publish(np2Fm(scnPn))
  return

def cb_X1(f):
  global bTm,bTmLat
  bTmLat=np.copy(bTm)
  print "bTm latched",bTmLat
  genpc=None
  try:
    genpc=rospy.ServiceProxy('/rovi/pshift_genpc',Trigger)
    req=TriggerRequest()
    genpc(req)      #will continue to callback cb_ps
  except rospy.ServiceException, e:
    print 'Genpc proxy failed:'+e
    pub_Y1.publish(False)
  return

def conv_ra(rx):
  return rx + 2 * 180 * (-1 if (rx > 180) else (1 if (rx <= -180) else 0))

def cb_X2(f):
  pp = PickingPose()
  pp.ok = False
  pp.x = 0
  pp.y = 0
  pp.z = 0
  pp.a = 0
  pp.b = 0
  pp.c = 0

  result = yodpy.loadPLY(scene_ply, scale="mm")
  retcode = result[0]
  scene = result[1]
  print('loadPLY retcode=',retcode)
  print('loadPLY scene=',scene)

  if retcode != 0:
    print "ERROR: X2 loadPLY() failed."
    pub_Y2.publish(pp)
    return

  # TODO
  #result = yodpy.match3D(scene)
  result = yodpy.match3D(scene,relSamplingDistance=0.03,keyPointFraction=0.1,minScore=0.11) # 0.25?
  #result = yodpy.match3D(scene,relSamplingDistance=0.03,keyPointFraction=0.05,minScore=0.11)
  #result = yodpy.match3D(scene,relSamplingDistance=0.05,keyPointFraction=0.1,minScore=0.11)

  retcode = result[0]
  transforms = result[1]
  quats = result[2]
  matchRates = result[3]
  print('match3D retcode=',retcode)

  if retcode != 0:
    print "ERROR: X2 match3D() failed."
    pub_Y2.publish(pp)
    return

  if (len(matchRates) <= 0):
    print "ERROR: X2 match3D() no match."
    pub_Y2.publish(pp)
    return

  for i, (transform, quat, matchRate) in enumerate(zip(transforms, quats, matchRates)):
    # NOTE:
    # 1. 'matchRates' are in descending order.
    # 2. 'quat' means a picking Pose.
    #    'quat' consists of 7 numpy.float64 values. They are x, y, z of Point position, and x, y, z, w of Quaternion orientation.
    # 3. Unit of x, y, z of Point position of 'quat' is meter.
    #
    #print('match3D quat=', quat)
    print('match3D matchRate=', matchRate)
    qx = quat[3]
    qy = quat[4]
    qz = quat[5]
    qw = quat[6]
    """
    if (qz < 0):
      qx = -qx
      qy = -qy
      qz = -qz
    """
    rad_euler = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))
    deg_euler_x = rad_euler[0] * 180 / np.pi
    deg_euler_y = rad_euler[1] * 180 / np.pi
    deg_euler_z = rad_euler[2] * 180 / np.pi
    #print "org quat.x=", quat[3], "quat.y=", quat[4], "quat.z=", quat[5], "quat.w=", quat[6]
    print "now quat.x=", qx, "quat.y=", qy, "quat.z=", qz, "quat.w=", qw
    print "rad_euler[0]=", rad_euler[0], "rad_euler[1]=", rad_euler[1], "rad_euler[2]=", rad_euler[2]
    print "deg_euler_x=", deg_euler_x, "deg_euler_y=", deg_euler_y, "deg_euler_z=", deg_euler_z
    ppx = quat[0] * 1000
    ppy = quat[1] * 1000
    ppz = quat[2] * 1000
    pprx = conv_ra(-180.0 - deg_euler_x)
    #pprx = deg_euler_x
    ppry = conv_ra(0 - deg_euler_y)
    #ppry = deg_euler_y
    pprz = conv_ra(-180.0 - deg_euler_z)
    #pprz = deg_euler_z
    print "****[", i, "]**** Picking Pose: x=", ppx, "y=", ppy, "z=", ppz, "roll=", pprx, "pitch=", ppry, "yaw=", pprz
    # TODO if orientation is upwards, reverse it.

    m2b44 = robot_rxyzabc_to_rt(ppx, ppy, ppz, rad_euler[0], rad_euler[1], rad_euler[2])

    radian = m2b44[0][2] * 0 + m2b44[1][2] * 0 + m2b44[2][2] * 1
    radian = radian / np.sqrt((np.power(m2b44[0][2], 2) + np.power(m2b44[1][2], 2) + np.power(m2b44[2][2], 2)) * (0 + 0 + 1))
    radian = np.arccos(radian)
    degree = radian * 180.0 / np.pi
    abs_deg = np.abs(degree)
    deg_threshold1 = 30.0
    deg_threshold2 = 30.0
    print "--[", i, "]-- angle between Picking Vector and Z-Axis=", degree
    if (abs_deg <= deg_threshold1):
      print "==[", i, "]== angle between Picking Vector and Z-Axis(", degree, ") <= ", deg_threshold1, "TODO cannot pick!!!!!!!!!!!!!!!!!!!"
      qx = -qx
      qy = -qy
      qz = -qz
      rad_euler = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))
      deg_euler_x = rad_euler[0] * 180 / np.pi
      deg_euler_y = rad_euler[1] * 180 / np.pi
      deg_euler_z = rad_euler[2] * 180 / np.pi
      deg_euler_x = rad_euler[0] * 180 / np.pi
      deg_euler_y = rad_euler[1] * 180 / np.pi
      deg_euler_z = rad_euler[2] * 180 / np.pi
      print "inverse quat.x=", qx, "quat.y=", qy, "quat.z=", qz, "quat.w=", qw
      print "now rad_euler[0]=", rad_euler[0], "rad_euler[1]=", rad_euler[1], "rad_euler[2]=", rad_euler[2]
      print "now deg_euler_x=", deg_euler_x, "deg_euler_y=", deg_euler_y, "deg_euler_z=", deg_euler_z
      pprx = conv_ra(-180.0 - deg_euler_x)
      #pprx = deg_euler_x
      ppry = conv_ra(0 - deg_euler_y)
      #ppry = deg_euler_y
      pprz = conv_ra(-180.0 - deg_euler_z)
      #pprz = deg_euler_z
      print "inversed. ****[", i, "]**** Picking Pose: x=", ppx, "y=", ppy, "z=", ppz, "roll=", pprx, "pitch=", ppry, "yaw=", pprz, "TODO more check for picking"
      pp.ok = True
      pp.x = ppx
      pp.y = ppy
      pp.z = ppz
      pp.a = pprx
      pp.b = ppry
      pp.c = pprz
      # determined!
      break
      break
    elif (abs_deg <= deg_threshold2):
      print "vv[", i, "]vv angle between Picking Vector and Z-Axis(", degree, ") <= ", deg_threshold2, "TODO cannot pick!!!!!!!!!!!!!!!!!!!"
      continue
    else:
      print "^^[", i, "]^^ angle between Picking Vector and Z-Axis(", degree, ") > ", deg_threshold2, "TODO more check for picking"
      pp.ok = True
      pp.x = ppx
      pp.y = ppy
      pp.z = ppz
      pp.a = pprx
      pp.b = ppry
      pp.c = pprz
      # determined!
      break

  # TODO determine a picking pose
  """
  global scnPn,modPn
  print "X2:ICP",modPn.shape,scnPn.shape
  icp=cv2.ppf_match_3d_ICP(100,Tolerance,Rejection,8)
  mp=modPn.astype(np.float32)
  sp=scnPn.astype(np.float32)
  ret,residu,RT=icp.registerModelToScene(mp,sp)
  print "Residue=",residu
  print RT
  pub_tf.publish(tflib.fromRT(RT))
  TR=np.linalg.inv(RT)
  P=np.vstack((scnPn.T,np.ones((1,len(scnPn)))))
  scnPn=np.dot(TR[:3],P).T
  pub_scf.publish(np2Fm(scnPn))
  """

  pub_Y2.publish(pp)

  return

def cb_tf(tf):
  global bTm
  bTm=tflib.toRT(tf)
  return

########################################################

rospy.init_node("solver",anonymous=True)
###Input topics
rospy.Subscriber("/robot/tf",Transform,cb_tf)
rospy.Subscriber("/rovi/ps_floats",numpy_msg(Floats),cb_ps)
rospy.Subscriber("/solver/X0",Bool,cb_X0)  #Clear scene
rospy.Subscriber("/solver/X1",Bool,cb_X1)  #Capture and genpc into scene
rospy.Subscriber("/solver/X2",Bool,cb_X2)  #Recognize work and calc picking pose

###Output topics
pub_tf=rospy.Publisher("/solver/tf",Transform,queue_size=1)
pub_scf=rospy.Publisher("/scene/floats",numpy_msg(Floats),queue_size=1)
pub_Y1=rospy.Publisher('/solver/Y1',Bool,queue_size=1)    #X1 done
pub_Y2=rospy.Publisher('/solver/Y2',PickingPose,queue_size=1)    #X2 done

###Transform
mTc=tflib.toRT(tflib.dict2tf(rospy.get_param('/robot/calib/mTc')))  # arM tip To Camera
#print "mTc=", mTc
bTmLat=np.eye(4).astype(float)  #robot RT when captured
bTm=np.eye(4).astype(float) 

###Globals
scnPn=P0()  #Scene points
modPn=P0()  #Model points
is_prepared = False

xmin = float(rospy.get_param('/volume_of_interest/xmin'))
xmax = float(rospy.get_param('/volume_of_interest/xmax'))
ymin = float(rospy.get_param('/volume_of_interest/ymin'))
ymax = float(rospy.get_param('/volume_of_interest/ymax'))
zmin = float(rospy.get_param('/volume_of_interest/zmin'))
zmax = float(rospy.get_param('/volume_of_interest/zmax'))
print "xmin=", xmin, "xmax=", xmax, "ymin=", ymin, "ymax=", ymax, "zmin=", zmin, "zmax=", zmax

try:
  prepare_model(os.environ['ROVI_PATH'] + '/wrs2018/model/Gear.stl')
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
