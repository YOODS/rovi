#!/usr/bin/env node

const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const fs = require('fs');

function topgm(img,fn) {
  let cap = new Uint8Array(img.data);
  return new Promise(function(resolve) {
    fs.writeFileSync(fn + '.pgm', 'P5\n' + img.width.toString() + ' ' + img.height.toString() + '\n' + '255\n');
    fs.appendFileSync(fn + '.pgm', cap);
    resolve(true);
  });
}

setImmediate(async function() {
  const rosNode = await ros.initNode('/imsave1');
  let subl = rosNode.subscribe('/rovi/camera/image_raw', sensor_msgs.Image, async function(src) {
    rosNode.unsubscribe('/rovi/camera/image_raw');
    await topgm(src, 'capt' + process.argv[2]);
    process.exit(0);
  });
});
