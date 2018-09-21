#!/usr/bin/env node

const ros = require('rosnodejs');
const geometry_msgs = ros.require('geometry_msgs').msg;
const sensor_msgs = ros.require('sensor_msgs').msg;
const std_msgs = ros.require('std_msgs').msg;
const rovi_msgs = ros.require('rovi').msg;
const EventEmitter = require('events').EventEmitter;
const Notifier = require('../script/notifier.js');
const net=require('net');

function xyz2quat(e) {
  let tf=Object.assign({},e);
  let k = Math.PI / 180 * 0.5;
  let cx = Math.cos(e.rotation.x * k);
  let cy = Math.cos(e.rotation.y * k);
  let cz = Math.cos(e.rotation.z * k);
  let sx = Math.sin(e.rotation.x * k);
  let sy = Math.sin(e.rotation.y * k);
  let sz = Math.sin(e.rotation.z * k);
  tf.rotation.x = cy * cz * sx - cx * sy * sz;
  tf.rotation.y = cy * sx * sz + cx * cz * sy;
  tf.rotation.z = cx * cy * sz - cz * sx * sy;
  tf.rotation.w = sx * sy * sz + cx * cy * cz;
  return tf;
}

setImmediate(async function(){
  const event=new EventEmitter();
  const rosNode=await ros.initNode('socket');

//Publisher
  const pub_tf=rosNode.advertise('/robot/tf', geometry_msgs.Transform);
  const pub_joint=rosNode.advertise('/robot/joint', geometry_msgs.Transform);
  const pub_solX0=rosNode.advertise('/solver/X0', std_msgs.Bool);
  const pub_solX1=rosNode.advertise('/solver/X1', std_msgs.Bool);
  const pub_solX2=rosNode.advertise('/solver/X2', std_msgs.Bool);
  const pub_gridX0=rosNode.advertise('/gridboard/X0', std_msgs.Bool);
  const pub_gridImg=rosNode.advertise('/gridboard/image_in', sensor_msgs.Image);

//Parameter notifier
  const param=new Notifier(rosNode,'/gridboard');
  param.on('change',function(key,val){
    console.log('param '+key+'='+val);
    pub_gridX0.publish(new std_msgs.Bool());
  });
  setTimeout(function(){ param.start(); },1000);

//Socket
  const server = net.createServer(function(conn){
    console.log('connection established');
    conn.on('data', function(data){   //data received from robot controller
      pub_solX1.publish(new std_msgs.Bool());
    });
    conn.on('close', function(){
      console.log('connection closed');
    });
    event.on('solve',function(tf){   //reply to robot controller
      conn.write('');
    });
  }).listen(3000);

//Subscriber (should be done after Looping started)
  setTimeout(function(){
    rosNode.subscribe('/robot/euler', geometry_msgs.Transform, async function(xyz){
      let qt=xyz2quat(xyz);
      pub_tf.publish(qt);
    });
    rosNode.subscribe('/solver/Y2', std_msgs.Bool, async function(tf){
      event.emit('solve',tf);
    });
    rosNode.subscribe('/rovi/left/image_rect', sensor_msgs.Image, async function(img){
      event.emit('image',img);
    });
    rosNode.subscribe('/gridboard/done', std_msgs.Bool, async function(f){
      event.emit('grid',f);
    });
  },1000);

//Looping image->grid
  while(true){
    await Promise.all([
      new Promise(function(resolve){
        event.once('image',function(img){
          pub_gridImg.publish(img);
          resolve(true);
        });
      }),
      new Promise(function(resolve){
        event.once('grid',async function(tf){
          resolve(true);
        });
      })
    ]);
  }
});
