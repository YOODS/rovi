#!/usr/bin/python
# coding: utf-8

import xml.etree.ElementTree as ET
import subprocess
import sys
import re
import rospy

from pprint import pprint
from collections import defaultdict

NODE_NM = 'genpc'

LOG_HEADER = '<cam_prm_reader> '

print (LOG_HEADER + "start")

if len(sys.argv) != 3:
    # print('Usage:calib_loader.py <camera_id> <camera_res>')
    print (LOG_HEADER + "error: wrong arguments.")
    sys.exit(-1)

camera_id = sys.argv[1]  # カメラid
camera_res = sys.argv[2]  # カメラ解像度
print (LOG_HEADER + "arg[1] camera_id=" + camera_id)
print (LOG_HEADER + "arg[2] camera_res=" + camera_res)
camera_res = camera_res.upper()

cam_xml = ""

print(LOG_HEADER + "camera xml read start.")
try:
    cmds = ["arv-tool-0.6"]
    if len(camera_id) > 0:
        cmds.append("-n")
        cmds.append(camera_id)
    cmds.append("genicam")
    print(LOG_HEADER + "cmds=" + ' '.join(cmds))
    cam_xml = subprocess.check_output(cmds)
    print(LOG_HEADER + "camera xml read finished.")
except subprocess.CalledProcessError:
    print(LOG_HEADER + 'error: arravis call failed.')
    sys.exit(-1)

if len(cam_xml) == 0:
    print(LOG_HEADER + "error: xml data read failed.")
    sys.exit(-1)

cam_xml_str = cam_xml.decode()
pattern = "YOODS Co,LTD.-YCAM3D-III-.*-->\n(.*)"
result = re.match(pattern, cam_xml_str, re.MULTILINE | re.DOTALL)
if result:
    print(LOG_HEADER + "xml comment area found. removed.")
    cam_xml_str = result.group(1)

result = re.match("(.*</RegisterDescription>)(.*)",cam_xml_str,re.MULTILINE| re.DOTALL)
if result:
    cam_xml_str = result.group(1)
    
print(LOG_HEADER + "xml parse start")
# tree = ET.parse("genicam.txt")
tree = ET.fromstring(cam_xml_str)

GEN_NS = '{http://www.genicam.org/GenApi/Version_1_0}'

ycam_serial_no = ""
ycam_major_ver = ""
ycam_minor_ver = ""
ycam_img_width = 0
ycam_img_height = 0
mat_K_rows = 0
mat_K_cols = 0
mat_R_rows = 0
mat_R_cols = 0
mat_T_rows = 0
mat_T_cols = 0


nested_dict = lambda: defaultdict(nested_dict)
cameras = nested_dict()


# ()で取りたい文字を
regex_cam_item = re.compile('^YCam_([\w]+)_cam([\w]+)_([\w]+)$')
regex_res_item = re.compile('^YCam_([\w]+)_([\w]+)$')
regex_calib_mat = re.compile('^([a-zA-Z]+)([0-9]+)$')

for node in tree.iter():

    name = node.attrib.get('Name')

    # Ycam_Major_VersionだけYcamで始まるので「Ycam_」も含める
    if not (name and name.startswith(('YCam_', 'Ycam_'))):
        continue

    val = None
    for item in node.iter(GEN_NS + 'Value'):
        # print("value=" + item.text)
        val = item.text

    if 'YCam_Serial_No' == name:
        for wkItem in node.iter(GEN_NS + 'Description'):
            ycam_serial_no = wkItem.text
    elif 'Ycam_Major_Version' == name: ycam_major_ver = int(val)
    elif 'YCam_Minor_Version' == name: ycam_minor_ver = int(val)
    elif 'YCam_Minor_Version' == name: ycam_minor_ver = int(val)
    elif 'YCam_Minor_Version' == name: ycam_minor_ver = int(val)
    elif 'YCam_K_Rows' == name: mat_K_rows = int(val)
    elif 'YCam_K_Cols' == name: mat_K_cols = int(val)
    elif 'YCam_R_Rows' == name: mat_R_rows = int(val)
    elif 'YCam_R_Cols' == name: mat_R_cols = int(val)
    elif 'YCam_T_Rows' == name: mat_T_rows = int(val)
    elif 'YCam_T_Cols' == name: mat_T_cols = int(val)
    else:
        match_result_ycam = regex_cam_item.match(name)
        if match_result_ycam:  # YCam_<Resolution>_cam<CameraNo>_<Key>
            ycam_res = match_result_ycam.group(1)
            ycam_no = match_result_ycam.group(2)
            ycam_key = match_result_ycam.group(3)
            if 'NDistortion' == ycam_key:
                cameras[ycam_res][ycam_no]['mat_D_rows'] = 1
                cameras[ycam_res][ycam_no]['mat_D_cols'] = int(val)
            else:  # 行列
                match_result_cam_mat = regex_calib_mat.match(ycam_key)
                if match_result_ycam:
                    mat_nm = match_result_cam_mat.group(1)
                    mat_no = match_result_cam_mat.group(2)
                    cameras[ycam_res][ycam_no][mat_nm][mat_no] = float(val)

        else:  # YCam_<Resolution>_<Key>
            match_result_res = regex_res_item.match(name)
            if match_result_res:
                ycam_res = match_result_res.group(1)
                ycam_key = match_result_res.group(2)

                if 'Width' == ycam_key: cameras[ycam_res][ycam_key] = int(val)
                elif 'Height' == ycam_key: cameras[ycam_res][ycam_key] = int(val)
                else:  # 行列
                    # print("res=" + ycam_res + " key=" + ycam_key)
                    match_result_cam_mat = regex_calib_mat.match(ycam_key)
                    if match_result_cam_mat:
                        mat_nm = match_result_cam_mat.group(1)
                        mat_no = match_result_cam_mat.group(2)
                        cameras[ycam_res][mat_nm][mat_no] = float(val)

print(LOG_HEADER + "xml parse finished.")

print(LOG_HEADER + "ros param regist start.")

rospy.set_param('/rovi/camera/serial_no', ycam_serial_no)
rospy.set_param('/rovi/camera/major_version', ycam_major_ver)
rospy.set_param('/rovi/camera/minor_version', ycam_minor_ver)

ycam_params = cameras[camera_res]
if not ycam_params:
    print (LOG_HEADER + "error: camera not found. res=" + camera_res)
    sys.exit(-1)

# print(dict(ycam_params))

ycam_width = ycam_params['Width']
ycam_height = ycam_params['Height']

ycam_params_l = ycam_params['0']
if not ycam_params_l:
    print (LOG_HEADER + "error: left camera not found. res=" + camera_res)
    sys.exit(-1)


def to_mat_array(mat_map, rows, cols):
    keys = sorted(mat_map)
    values = []
    for key in keys:
        values.append(mat_map[key])

    return values

rospy.set_param('/rovi/left/' + NODE_NM + '/Width', ycam_width)
rospy.set_param('/rovi/left/' + NODE_NM + '/Height', ycam_height)
rospy.set_param('/rovi/left/' + NODE_NM + '/K_Rows', mat_K_rows)
rospy.set_param('/rovi/left/' + NODE_NM + '/K_Cols', mat_K_cols)
rospy.set_param('/rovi/left/' + NODE_NM + '/K', to_mat_array(ycam_params_l['K'], mat_K_rows, mat_K_cols))
rospy.set_param('/rovi/left/' + NODE_NM + '/R_Rows', mat_R_rows)
rospy.set_param('/rovi/left/' + NODE_NM + '/R_Cols', mat_R_cols)
rospy.set_param('/rovi/left/' + NODE_NM + '/R', [1., 0., 0., 0., 1., 0., 0., 0., 1.])
rospy.set_param('/rovi/left/' + NODE_NM + '/T_Rows', mat_T_rows)
rospy.set_param('/rovi/left/' + NODE_NM + '/T_Cols', mat_T_cols)
rospy.set_param('/rovi/left/' + NODE_NM + '/T', [0., 0., 0.])
rospy.set_param('/rovi/left/' + NODE_NM + '/D_Rows', ycam_params_l['mat_D_rows'])
rospy.set_param('/rovi/left/' + NODE_NM + '/D_Cols', ycam_params_l['mat_D_cols'])
rospy.set_param('/rovi/left/' + NODE_NM + '/D', to_mat_array(ycam_params_l['D'], ycam_params_l['mat_D_rows'], ycam_params_l['mat_D_cols']))


ycam_params_r = ycam_params['1']
if not ycam_params_r:
    print (LOG_HEADER + "error: right camera not found. res=" + camera_res)
    sys.exit(-1)

rospy.set_param('/rovi/right/' + NODE_NM + '/Width', ycam_width)
rospy.set_param('/rovi/right/' + NODE_NM + '/Height', ycam_height)
rospy.set_param('/rovi/right/' + NODE_NM + '/K_Rows', mat_K_rows)
rospy.set_param('/rovi/right/' + NODE_NM + '/K_Cols', mat_K_cols)
rospy.set_param('/rovi/right/' + NODE_NM + '/K', to_mat_array(ycam_params_r['K'], mat_K_rows, mat_K_cols))
rospy.set_param('/rovi/right/' + NODE_NM + '/R_Rows', mat_R_rows)
rospy.set_param('/rovi/right/' + NODE_NM + '/R_Cols', mat_R_cols)
rospy.set_param('/rovi/right/' + NODE_NM + '/R', to_mat_array(ycam_params['R'], mat_R_rows, mat_R_cols))
rospy.set_param('/rovi/right/' + NODE_NM + '/T_Rows', mat_T_rows)
rospy.set_param('/rovi/right/' + NODE_NM + '/T_Cols', mat_T_cols)
rospy.set_param('/rovi/right/' + NODE_NM + '/T', to_mat_array(ycam_params['T'], mat_T_rows, mat_T_cols))
rospy.set_param('/rovi/right/' + NODE_NM + '/D_Rows', ycam_params_r['mat_D_rows'])
rospy.set_param('/rovi/right/' + NODE_NM + '/D_Cols', ycam_params_r['mat_D_cols'])
rospy.set_param('/rovi/right/' + NODE_NM + '/D', to_mat_array(ycam_params_r['D'], ycam_params_r['mat_D_rows'], ycam_params_r['mat_D_cols']))

print(LOG_HEADER + "ros param regist finished.")
print(LOG_HEADER + "finished")
