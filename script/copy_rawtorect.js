#!/usr/bin/env node

const NS = '/narrow_stereo_textured';

const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;

setImmediate(async function() {
  const rosNode = await ros.initNode(NS);
  const rect_L = rosNode.advertise(NS + '/left/image_rect', sensor_msgs.Image);
  const rect_R = rosNode.advertise(NS + '/right/image_rect', sensor_msgs.Image);
  rosNode.subscribe(NS + '/left/image_raw', sensor_msgs.Image, async function(img) {
    rect_L.publish(img);
  });
  rosNode.subscribe(NS + '/right/image_raw', sensor_msgs.Image, async function(img) {
    rect_R.publish(img);
  });
});
