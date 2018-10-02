#!/usr/bin/env node

const ros = require('rosnodejs');
const std_msgs = ros.require('std_msgs').msg;
const EventEmitter = require('events').EventEmitter;
const net = require('net');

// TODO 撮影結果もY1で捕まえてロボットに返す
// そしてそこからX2を打つ

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
      pub_solX0.publish(new std_msgs.Bool());
      pub_solX1.publish(new std_msgs.Bool());
    });
    conn.on('close', function() {
      console.log('robot controller connection closed');
    });
    event.on('solve', function(tf) {   //reply to robot controller
      conn.write(''); // TODO ピッキング位置姿勢
    });
  }).listen(3000);

//Subscriber
  rosNode.subscribe('/solver/Y2', std_msgs.Bool, async function(tf) {
    event.emit('solve', tf);
  });

});
