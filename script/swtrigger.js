#!/usr/bin/env node

'use strict';
const ros = require('rosnodejs');
const std_msgs = ros.require('std_msgs').msg;
const sensor_msgs = ros.require('sensor_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;

var camTriggerClient=null;
var camTriggerRequest=null;
var camTriggerWDT=null;
function camTrigger(){
	camTriggerClient.call(camTriggerRequest).then(function(resp){});
	camTriggerWDT=setTimeout(function(){
		camTrigger();
	},100);
}
function camOK(){
	clearTimeout(camTriggerWDT);
}

ros.initNode('/rovi/swtriggerjs').then((rosNode)=>{
	let sub=rosNode.subscribe('/camera/image_raw',sensor_msgs.Image,(img)=>{
		camOK();
		ros.log.info('image_raw subscribed');
		camTrigger();
	});
	var cl=rosNode.serviceClient('camera/queue',std_srvs.Trigger,{persist:true});
	rosNode.waitForService(cl.getService(),2000).then(function(available){
		if(!available){
			ros.log.info('Service camera/queue not available');
			return;
		}
		camTriggerClient=cl;
		camTriggerRequest=new std_srvs.Trigger.Request();
		camTrigger();
	});
});
