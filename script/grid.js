#!/usr/bin/env node

'use strict';

const ros = require('rosnodejs');
const std_msgs = ros.require('std_msgs').msg;
const geometry_msgs = ros.require('geometry_msgs').msg;
const sensor_msgs = ros.require('sensor_msgs').msg;
const rovi_srvs = ros.require('rovi').srv;
const std_srvs = ros.require('std_srvs').srv;

var camTriggerClient = null;
var camTriggerRequest = null;
var camTriggerWDT = null;

function camTrigger() {
  camTriggerClient.call(camTriggerRequest).then(function(resp) {});
  camTriggerWDT = setTimeout(camTrigger, 100);
}

function camOK() {
  clearTimeout(camTriggerWDT);
}

ros.initNode('/rovi/grid').then((rosNode) => {
  var cl1 = rosNode.serviceClient('GetGrid', rovi_srvs.GetGrid, { persist: true });
  rosNode.waitForService(cl1.getService(), 2000).then(function(available) {
    if (!available) {
      ros.log.info('Service GetGrid not available');
      return;
    }
    var getGridClient = cl1;
    var getGridRequest = new rovi_srvs.GetGrid.Request();
    let pub = rosNode.advertise('/rovi/GetGrid/image', sensor_msgs.Image);
    let sub = rosNode.subscribe('/camera/image_raw', sensor_msgs.Image, (img) => {
      camOK();
      getGridRequest.img = img;
      getGridClient.call(getGridRequest).then(function(resp) {
        pub.publish(resp.img);
        ros.log.info('Response grid ' + JSON.stringify(resp.grid.length));
        if (resp.grid.length > 0) {
          
        }
        camTrigger();
      });
    });
  });
  var cl2 = rosNode.serviceClient('camera/queue', std_srvs.Trigger, { persist: true });
  rosNode.waitForService(cl2.getService(), 2000).then(function(available) {
    if (!available) {
      ros.log.info('Service camera/queue not available');
      return;
    }
    camTriggerClient = cl2;
    camTriggerRequest = new std_srvs.Trigger.Request();
    camTrigger();
  });
});
