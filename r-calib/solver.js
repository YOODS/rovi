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

function quatConcat(q1,q2){
}

setImmediate(async function(){
  const rosNode=await ros.initNode('solver');
  const param=new Notifier(rosNode,'/gridboard');
  const event=new EventEmitter();
  const get_tf=client(rosNode,'/compute_effector_camera_quick',visp_srvs.compute_effector_camera_quick);
  const pub1=rosNode.advertise('/gridboard/image_in', sensor_msgs.Image);
  const pub2=rosNode.advertise('/gridboard/X0', std_msgs.Empty);
  const pub10=rosNode.advertise('/solver/cTs', geometry_msgs.Transform);

  param.on('change',function(key,val){
    pub2.publish(new std_msgs.Empty());
  });
  setTimeout(async function(){ param.start(); },1000);

  let bTm=new geometry_msgs.Transform();
  let mTc=new geometry_msgs.Transform();
  let cTs=new geometry_msgs.Transform();
  rosNode.subscribe('/robot/tf', geometry_msgs.Transform, async function(tf){
    pub2.publish();
  });
  let c2o=new visp_msgs.TransformArray();
  let w2e=new visp_msgs.TransformArray();
  rosNode.subscribe('solver/X0', std_msgs.Empty, async function(tf){
    c2o = new visp_msgs.TransformArray();
    w2e = new visp_msgs.TransformArray();
    ros.log.info('pose cleared');
  });
  rosNode.subscribe('solver/X1', std_msgs.Empty, async function(){
    event.once('pose', async function(cTs){
      ros.log.info('object pose['+c2o.transforms.length+']\n'+JSON.stringify(cTs));
      c2o.transforms.push(cTs);
      ros.log.info('robot pose['+w2e.transforms.length+']\n'+JSON.stringify(bTm));
      w2e.transforms.push(bTm);
    });
  });
  rosNode.subscribe('solver/X2', std_msgs.Empty, async function(){
    ros.log.info('calcurate');
    let req=new visp_srvs.compute_effector_camera_quick.Request();
    req.camera_object=c2o;
    req.world_effector=w2e;
    let res;
    try {
      res=await get_tf.call(req);
      rosNode.setParam('/robot/calib',res.effector_camera);
      ros.log.info('TF='+JSON.strigify(res.effector_camera));
    }
    catch(err){
      ros.log.error(err);
    }
  });
  rosNode.subscribe('/rovi/left/image_rect', sensor_msgs.Image, async function(img){
    event.emit('img',img);
  });
  rosNode.subscribe('/gridboard/tf', geometry_msgs.Transform, async function(tf){
    event.emit('tf',cTs=tf);
  });

  while(true){
    await Promise.all([
      new Promise(function(resolve){
        event.once('img',async function(img){
          pub1.publish(img);
        });
        event.once('tf',function(){
          resolve(true);
        });
      })
    ]);
  }
});
