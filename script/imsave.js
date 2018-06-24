#!/usr/bin/env node

const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const fs = require('fs');
const topic = 'image_raw';

function topgm(img,fn) {
  let cap = new Uint8Array(img.data);
  return new Promise(function(resolve) {
    fs.writeFileSync(fn + '.pgm', 'P5\n' + img.width.toString() + ' ' + img.height.toString() + '\n' + '255\n');
    fs.appendFileSync(fn + '.pgm', cap);
    resolve(true);
  });
}

setImmediate(async function() {
  const rosNode = await ros.initNode('/imsave');
  let lf = 0;
  let rf = 0;
  let subl = rosNode.subscribe('/rovi/left/' + topic, sensor_msgs.Image, async function(src) {
    rosNode.unsubscribe('/rovi/left/' + topic);
    lf++;
    await topgm(src, 'left' + process.argv[2]);
    lf++;
    if (rf >= 2) process.exit(0);    
  });
  let subr = rosNode.subscribe('/rovi/right/' + topic, sensor_msgs.Image, async function(src) {
    rosNode.unsubscribe('/rovi/right/' + topic);
    rf++;
    await topgm(src, 'right' + process.argv[2]);
    rf++;
    if (lf >= 2) process.exit(0);
  });
});
