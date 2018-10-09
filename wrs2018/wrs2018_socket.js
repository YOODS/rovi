#!/usr/bin/env node

const ros = require('rosnodejs');
const std_msgs = ros.require('std_msgs').msg;
const rovi_msgs = ros.require('rovi').msg;
const EventEmitter = require('events').EventEmitter;
const net = require('net');

setImmediate(async function() {
  const event = new EventEmitter();
  const rosNode = await ros.initNode('socket');

//Publisher
  const pub_solX0 = rosNode.advertise('/solver/X0', std_msgs.Bool);
  const pub_solX1 = rosNode.advertise('/solver/X1', std_msgs.Bool);
  const pub_solX2 = rosNode.advertise('/solver/X2', std_msgs.Bool);

//Socket
  const server = net.createServer(function(conn) {
    console.log('robot controller connection established');
    conn.on('data', function(data) {   //data received from robot controller
      console.log('from robot controller data[' + data + ']');
      if (String(data).charAt(0) === 'H') {
        //pub_solX0.publish(new std_msgs.Bool());
        pub_solX1.publish(new std_msgs.Bool());
      }
    });
    conn.on('close', function() {
      console.log('robot controller connection closed');
    });
    event.on('capt', function(success) {   // reply capture result to robot controller
      if (success) {
        conn.write('OK\x0d');
        pub_solX2.publish(new std_msgs.Bool());
      }
      else {
        conn.write('NG\x0d');
      }
    });
    event.on('solve', function(pp) {   //reply picking pose to robot controller
      console.log('got Y2 solve. pp.ok=' + pp.ok);
      if (pp.ok) {
        okstr = 'OK\x0d(' + pp.x.toFixed(3) + ',' + pp.y.toFixed(3) + ',' + pp.z.toFixed(3) + ',' + pp.a.toFixed(3) + ',' + pp.b.toFixed(3) + ',' + pp.c.toFixed(3) + ')(7,0)\x0d';
        console.log(okstr);
        conn.write(okstr);
      }
      else {
        conn.write('NG\x0d');
      }
    });
  }).listen(3000);

//Subscriber
  rosNode.subscribe('/solver/Y1', std_msgs.Bool, async function(isok) {
    event.emit('capt', isok.data);
  });
  rosNode.subscribe('/solver/Y2', rovi_msgs.PickingPose, async function(picking_pose) {
    event.emit('solve', picking_pose);
  });

});
