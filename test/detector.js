#!/usr/bin/env node

const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const rovi_srvs = ros.require('rovi').srv;
const EventEmitter = require('events').EventEmitter;

setImmediate(async function() {
  const rosNode = await ros.initNode('detector_node');
  const circle = rosNode.serviceClient('/circle_detector/do', rovi_srvs.Detect2D, { persist: true });
  if (!await rosNode.waitForService(circle.getService(), 2000)){
    ros.log.error('circle service not available');
    return;
  }

  let image_queue=null;
  async function callDetector(){
    let req = new rovi_srvs.Detect2D.Request();
    let seq = image_queue.header.seq;
    req.img = image_queue;
    let res = await circle.call(req);
    console.log(res.scene)
    if(seq!=image_queue.header.seq) setImmediate(callDetector);
    else image_queue=null;
  }

//  const sub = rosNode.subscribe('/rovi/left/image_rect', sensor_msgs.Image, async (src)=>{
  const sub = rosNode.subscribe('/camera/image_raw', sensor_msgs.Image, async (src)=>{
    let f=image_queue==null;
    image_queue=src;
    if(f) callDetector();
  });
});
