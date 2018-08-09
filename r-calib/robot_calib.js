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
const Notifier = require('../script/notifier.js');

function client(ros,node,srv){
  const c=ros.serviceClient(node,srv,{ persist: true });
  if (!ros.waitForService(c.getService(),1000)){
    ros.log.error(node+' not available');
    throw new Error('no service available');
  }
  return c;
}

function xyz2quat(e) {
  let tf=new geometry_msgs.Transform();
  tf.translation.x=parseFloat(e[0]);
  tf.translation.y=parseFloat(e[1]);
  tf.translation.z=parseFloat(e[2]);
  let k = Math.PI / 180 * 0.5;
  let cx = Math.cos(e[3] * k);
  let cy = Math.cos(e[4] * k);
  let cz = Math.cos(e[5] * k);
  let sx = Math.sin(e[3] * k);
  let sy = Math.sin(e[4] * k);
  let sz = Math.sin(e[5] * k);
  tf.rotation.x = cy * cz * sx - cx * sy * sz;
  tf.rotation.y = cy * sx * sz + cx * cz * sy;
  tf.rotation.z = cx * cy * sz - cz * sx * sy;
  tf.rotation.w = sx * sy * sz + cx * cy * cz;
  return tf;
}

setImmediate(async function(){
  const rosNode=await ros.initNode('robot_calib');
  const param=new Notifier(rosNode,'/gridboard');
  const event=new EventEmitter();
  const get_pose=client(rosNode,'/gridboard',rovi_srvs.ImageFilter);
  const res_grid=client(rosNode,'/gridboard/reload',std_srvs.Trigger);
  const get_tf=client(rosNode,'/compute_effector_camera_quick',visp_srvs.compute_effector_camera_quick);

  param.on('change',function(key,val){
    const req=new std_srvs.Trigger.Request();
    res_grid.call(req);
  });
  setTimeout(async function(){ param.start(); },1000);

  let c2o=new visp_msgs.TransformArray();
  let w2e=new visp_msgs.TransformArray();
  process.stdin.on('data',async function(chk){
    let tok=new String(chk).split(' ');
    switch(tok[0].trim()){
    case 'c':
      ros.log.info('calcurate');
      let req=new visp_srvs.compute_effector_camera_quick.Request();
      req.camera_object=c2o;
      req.world_effector=w2e;
      let res;
      try {
        res=await get_tf.call(req);
        rosNode.setParam('/robot_calib/tf',res.effector_camera);
        ros.log.info('TF='+JSON.strigify(res.effector_camera));
      }
      catch(err){
        ros.log.error(err);
      }
    case 'r':
      c2o = new visp_msgs.TransformArray();
      w2e = new visp_msgs.TransformArray();
      ros.log.info('pose cleared');
      break;
    default:
      if(tok.length<6 || parseFloat(tok[0])=='NaN'){
        ros.log.warn('Format error');
        break;
      }
      let trm=xyz2quat(tok);
      event.once('pose', async function(pose){
        let tro = new geometry_msgs.Transform();
        tro.translation.x = pose.position.x;
        tro.translation.y = pose.position.y;
        tro.translation.z = pose.position.z;
        tro.rotation = pose.orientation;
        ros.log.info('object pose['+c2o.transforms.length+']\n'+JSON.stringify(tro));
        c2o.transforms.push(tro);
        ros.log.info('robot pose['+w2e.transforms.length+']\n'+JSON.stringify(trm));
        w2e.transforms.push(trm);
      });
    }
  });

  rosNode.subscribe('/rovi/left/image_rect', sensor_msgs.Image, async function(src){
    event.emit('img',src);
  });
  rosNode.subscribe('/gridboard/pose', geometry_msgs.Pose, async function(src){
    event.emit('pose',src);
  });

  while(true){
    await Promise.all([
      new Promise(function(resolve){
        event.once('img',async function(img){
          let req=new rovi_srvs.ImageFilter.Request();
          req.img=img;
          await get_pose.call(req);
          resolve(true);
        });
      })
    ]);
  }
});
