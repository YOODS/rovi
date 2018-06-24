#!/usr/bin/env node

const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const geometry_msgs = ros.require('geometry_msgs').msg;
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const rovi_srvs = ros.require('rovi').srv;
const visp_msgs = ros.require('visp_hand2eye_calibration').msg;
const visp_srvs = ros.require('visp_hand2eye_calibration').srv;
const EventEmitter = require('events').EventEmitter;

setImmediate(async function() {
  const rosNode = await ros.initNode('robot_calib');
  const evs = new EventEmitter();
  const cl_grid = rosNode.serviceClient('/gridboard', rovi_srvs.GetGrid, { persist: true });
  if (!rosNode.waitForService(cl_grid.getService(), 1000)) {
    ros.log.error('grid_node not available');
    return;
  }
  const cl_calib = rosNode.serviceClient('/compute_effector_camera_quick', visp_srvs.compute_effector_camera_quick, { persist: true });
  if (!rosNode.waitForService(cl_calib.getService(), 1000)) {
    ros.log.error('calibrator::compute not available');
    return;
  }
  const cl_reset = rosNode.serviceClient('/reset', visp_srvs.reset, { persist: true});
  if (!rosNode.waitForService(cl_reset.getService(), 1000)) {
    ros.log.error('calibrator::reset not available');
    return;
  }
  let c2o = new visp_msgs.TransformArray();
  let w2e = new visp_msgs.TransformArray();
  const svc1 = rosNode.advertiseService('/robot_calib/pose', rovi_srvs.SetPose, (req, res) => {
ros.log.info('robot_pose:' + JSON.stringify(req.pose));
    let trm = new geometry_msgs.Transform();
    trm.translation = req.pose.position;
    trm.rotation = req.pose.orientation;
    return new Promise(function(resolve) {
      evs.once('pose', async function(pose) {
ros.log.info('object_pose:' + JSON.stringify(pose));
        let tro = new geometry_msgs.Transform();
        tro.translation.x = pose.position.x;
        tro.translation.y = pose.position.y;
        tro.translation.z = pose.position.z;
        tro.rotation = pose.orientation;
        c2o.transforms.push(tro);
        w2e.transforms.push(trm);
        resolve(true);
      });
    });
  });
  let pub_grid = rosNode.advertise('/robot_calib/image', sensor_msgs.Image);
  let sub = rosNode.subscribe('/left/image_rect', sensor_msgs.Image, async (src) => {
    let req = new rovi_srvs.GetGrid.Request();
    req.img = src;
    let res = await cl_grid.call(req);
    pub_grid.publish(res.img);
    evs.emit('pose', res.pose);
  });
  const svc2 = rosNode.advertiseService('/robot_calib/reload', std_srvs.Trigger, async (req, res) => {
    let rq = new visp_srvs.reset.Request();
    await cl_reset.call(rq);
    rq = new visp_srvs.compute_effector_camera_quick.Request();
    rq.camera_object = c2o;
    rq.world_effector = w2e;
    let rs;
    try {
      rs = await cl_calib.call(rq);
      res.message = JSON.stringify(rs.effector_camera);
      res.success = true;
      rosNode.setParam('/robot_calib/tf', rs.effector_camera);
    }
    catch(err) {
      res.message = err;
      res.success = false;
    }
    c2o = new visp_msgs.TransformArray();
    w2e = new visp_msgs.TransformArray();
    return true;
  });
});
