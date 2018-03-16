#!/usr/bin/env node

const ros=require('rosnodejs');
const sensor_msgs=ros.require('sensor_msgs').msg;
const sens=require('./ycam1h.js');
const std_msgs=ros.require('std_msgs').msg;
const std_srvs=ros.require('std_srvs').srv;
let perf=ros.Time;
perf.msec=function(){
	let t=this.now();
	return t.secs*1e3+t.nsecs*1e-6;
}

ros.initNode('/test').then((rosNode)=>{
	const pub_L=rosNode.advertise('/cam_L/image', sensor_msgs.Image);
	const pub_ct=rosNode.advertise('/cam_L/interval',std_msgs.Float32);
	let msg_ct=new std_msgs.Float32();
	let msg_cto=perf.msec();
	rosNode.getParam('/cam_L/ID')
	.then(function(id){
		let ev=sens.start(rosNode,id);
		ev.on('cam_L',function(img){
			pub_L.publish(img);
			let tnow=perf.msec();
console.log('now:'+tnow);
			msg_ct.data=tnow-msg_cto;
			msg_cto=tnow;
			pub_ct.publish(msg_ct);
		});
	});
	const svc1=rosNode.advertiseService('/cam_L/extern',std_srvs.SetBool, (req,res)=>{
		sens.set({'TriggerMode':req.data? 'On':'Off','AcquisitionFrameRate':10.0});
		return res.success=true;
	});
});
