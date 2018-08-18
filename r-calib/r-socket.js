#!/usr/bin/env node

const ros = require('rosnodejs');
const geometry_msgs = ros.require('geometry_msgs').msg;
const std_msgs = ros.require('std_msgs').msg;
const EventEmitter = require('events').EventEmitter;
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
  const rosNode=await ros.initNode('r-socket');
  const pub=rosNode.advertise('/robot/tf', geometry_msgs.Transform);
  const event=new EventEmitter();

  rosNode.subscribe('/robot/euler', geometry_msgs.Transform, async function(xyz){
    let tf=xyz2quat(xyz);
    pub.publish(tf);
  });
  rosNode.subscribe('/solver/tf', geometry_msgs.Transform, async function(tf){
    event.emit('solve',tf);
  });

  const server = net.createServer(function(conn){
    console.log('connection established');
    conn.on('data', function(data){   //data received from robot controller
//      pub.publish(tf);
      conn.write('server -> Repeating: ' + data);
    });
    conn.on('close', function(){
      console.log('connection closed');
    });
    event.on('solve',function(tf){   //reply to robot controller
      conn.write('');
    });
  }).listen(3000);
});
