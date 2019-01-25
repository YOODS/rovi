#!/usr/bin/env node

const ros = require('rosnodejs');
const geometry_msgs = ros.require('geometry_msgs').msg;
const sensor_msgs = ros.require('sensor_msgs').msg;
const std_msgs = ros.require('std_msgs').msg;
const rovi_msgs = ros.require('rovi').msg;
const EventEmitter = require('events').EventEmitter;
const Notifier = require('../script/notifier.js');
const net=require('net');

require('date-utils')

for (let i = 0; i < process.argv.length; i++) {
  console.log('argv['+i.toString()+'] = '+process.argv[i]);
}
const rOrder=process.argv[2]
if (rOrder=="cba") {
  console.log('rOrderCBA='+rOrder);
} else {
  console.log('rOrderABC='+rOrder);
}

function toCoords(data) {
// data format is '***(X,Y,Z,A,B,C)***\n'.
  const str = data.toString();
  const ary = str.replace(/\).*/g, ']').replace(/.*\(/, '[').replace(/E\+/g, 'E').replace(/\+/g, '');
//  console.log('ary={' + ary + '}');
  let coords = [];
  try {
    coords = JSON.parse('[' + ary + ']');
  }
  catch(e) {
    console.log("error " + e);
  }
  console.log('coord='+coords.toString());
  let euler=new geometry_msgs.Transform();
  euler.translation.x=coords[0][0];
  euler.translation.y=coords[0][1];
  euler.translation.z=coords[0][2];
  euler.rotation.x=coords[0][3];
  euler.rotation.y=coords[0][4];
  euler.rotation.z=coords[0][5];
  euler.rotation.w=1;
//  console.log(JSON.stringify(euler));
  return euler
}

function xyz2quat(e) {
  let tf=Object.assign({},e);
  let k = Math.PI / 180 * 0.5;
  let cx = Math.cos(e.rotation.x * k);
  let cy = Math.cos(e.rotation.y * k);
  let cz = Math.cos(e.rotation.z * k);
  let sx = Math.sin(e.rotation.x * k);
  let sy = Math.sin(e.rotation.y * k);
  let sz = Math.sin(e.rotation.z * k);
  if (rOrder=="cba") {
    tf.rotation.x = cy * cz * sx - cx * sy * sz;
    tf.rotation.y = cy * sx * sz + cx * cz * sy;
    tf.rotation.z = cx * cy * sz - cz * sx * sy;
    tf.rotation.w = sx * sy * sz + cx * cy * cz;
  } else {
    tf.rotation.x = sx * cy * cz - cx * sy * sz;
    tf.rotation.y = cx * sy * cz + sx * cy * sz;
    tf.rotation.z = cx * cy * sz - sx * sy * cz;
    tf.rotation.w = cx * cy * cz + sx * sy * sz;
  }
  return tf;
}

setImmediate(async function(){
  const event=new EventEmitter();
  const rosNode=await ros.initNode('socket');
//Subscribers
  rosNode.subscribe('/solver/Y1',std_msgs.Bool,async function(ret) {
    let now = new Date();
    console.log('##### receive Y1 --> emit caught' +now.toFormat('YYYY_MM_DD_HH24MISS'));
    event.emit('caught',ret.data);
  });

  rosNode.subscribe('/solver/Y2', rovi_msgs.PickingPose, async function(picking_pose) {
    let now = new Date();
    console.log('##### receive Y2 --> emit solved' +now.toFormat('YYYY_MM_DD_HH24MISS'));
    event.emit('solved', picking_pose);
  });

  rosNode.subscribe('/solver/mTs',geometry_msgs.Transform,async function(tf) {
    let now = new Date();
    console.log('##### receive mTs --> emit position' +now.toFormat('YYYY_MM_DD_HH24MISS'));
    event.emit('position',tf);
  });

//Publisher
  const pub_tf=rosNode.advertise('/robot/tf', geometry_msgs.Transform);
  const pub_solX0=rosNode.advertise('/solver/X0', std_msgs.Bool);    // Clear
  const pub_solX1=rosNode.advertise('/solver/X1', std_msgs.Bool);    // Capture
  const pub_solX2=rosNode.advertise('/solver/X2', std_msgs.Bool);    // Calc
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
  let cnt=0;
  const server = net.createServer(function(conn){
    console.log('r_socket connected');
    conn.setTimeout(60000); //60秒

    let msg='';
    conn.on('data', function(data){
      msg+=data.toString();
      console.log('msg='+msg);
      if(msg.indexOf('(')*msg.indexOf(')')<0) return;

      if(msg.startsWith('P1')){
        //publish /robot/tf, no reply
        let tf=toCoords(msg);
        let qt=xyz2quat(tf);
        pub_tf.publish(qt);
      }
      else if(msg.startsWith('X0')){
        //publish /solver/X0
        pub_solX0.publish(new std_msgs.Bool());
        conn.write('OK\n');
        cnt=0;
      }
      else if(msg.startsWith('X1')){
        let now = new Date();
	console.log('##### call_X1 ' +now.toFormat('YYYY_MM_DD_HH24MISS'));

        //publish /robot/tf
        let euler=toCoords(msg);
        let qt=xyz2quat(euler);
        pub_tf.publish(qt);
        event.once('position',function(tf){
          console.log('['+cnt.toString()+']position');
          //publish /solver/X1
          pub_solX1.publish(new std_msgs.Bool());
        });
        event.once('caught',function(ret){  // send capture result
          now = new Date();
	  console.log('##### caught start ' +now.toFormat('YYYY_MM_DD_HH24MISS'));

          console.log('['+cnt.toString()+']caught: '+((ret) ? 'OK' : 'NG'));
          if (ret) {
            conn.write('OK\n');
            cnt+=1;
          } else {
            conn.write('NG\n');
            conn.write('999');
          }
        });
      }
      else if(msg.startsWith('X2')){
        //publish /solver/X2
        let now = new Date();
	console.log('##### call_X2 ' +now.toFormat('YYYY_MM_DD_HH24MISS'));

        pub_solX2.publish(new std_msgs.Bool());
	if (rOrder=="cba") {
	  event.once('solved', function(pp) {   //reply picking pose to robot controller

            now = new Date();
	    console.log('##### solved start ' +now.toFormat('YYYY_MM_DD_HH24MISS'));

            console.log('got Y2 solve. pp.ok=' + pp.ok);
            if (pp.ok) {
              okstr = 'OK\x0d(' + pp.x.toFixed(3) + ',' + pp.y.toFixed(3) + ',' + pp.z.toFixed(3) + ',' + pp.a.toFixed(3) + ',' + pp.b.toFixed(3) + ',' + pp.c.toFixed(3) + ')(7,0)\x0d';
              console.log(okstr);
              conn.write(okstr);
            }
            else {
              if (pp.errorreason == "SPARSE") {
                console.log('[2]send robot SKIP...\x0d');
                conn.write('SKIP\x0d');
              }
              else if (pp.errorreason == "CL") {
                console.log('[2]send robot CL...\x0d');
                if (pp.x>0) {
                  clstr = 'CL\x0d(' + pp.x.toFixed(3) + ',' + pp.y.toFixed(3) + ',' + pp.z.toFixed(3) + ',0,0,0)(7,0)\x0d(' + pp.a.toFixed(3) + ',' + pp.b.toFixed(3) + ',' + pp.c.toFixed(3) + ',0,0,0)(7,0)\x0d';
                } else {
                  clstr = 'CL\x0d(' + pp.a.toFixed(3) + ',' + pp.b.toFixed(3) + ',' + pp.c.toFixed(3) + ',0,0,0)(7,0)\x0d(' + pp.x.toFixed(3) + ',' + pp.y.toFixed(3) + ',' + pp.z.toFixed(3) + ',0,0,0)(7,0)\x0d';
                }
                console.log(clstr);
                conn.write(clstr);
              }
              else {
                console.log('[2]send robot NG...\x0d');
                conn.write('NG\x0d');
              }
            }
            console.log('[2]sent!\x0d');
          });
        }else{
	  event.once('solved', function(pp) {   //reply picking pose to robot controller
            console.log('got Y2 solve. pp.ok=' + pp.ok);
            if (pp.ok) {
              conn.write('OK\n');
              let tstr="'"+pp.x.toFixed(3)+"''"+pp.y.toFixed(3)+"''"+pp.z.toFixed(3)+"'";
              let rstr="'"+pp.a.toFixed(3)+"''"+pp.b.toFixed(3)+"''"+pp.c.toFixed(3)+"'";
              console.log('translation '+tstr);
              console.log('rotation '+rstr);
              conn.write(tstr+rstr);
            } else {
              conn.write('NG\n');
              conn.write('999');
            }
          });
          console.log('[2]sent!\x0d');
        }
      }
      msg='';
      return;
    });
    conn.on('close', function(){
      console.log('connection closed');
    });
    conn.on('timeout',function(){
      event.removeAllListeners();
      conn.write('NG\n'+"'408'"); //Request timeout
      console.log('request timeout');
      conn.destroy();
    });
  }).listen(3000);

//Subscriber (should be done after Looping started)
  setTimeout(function(){
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
        event.once('grid',async function(f){
          resolve(true);
        });
      })
    ]);
  }
});
