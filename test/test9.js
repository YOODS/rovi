#!/usr/bin/env node

const ros=require('rosnodejs');
const sensor_msgs=ros.require('sensor_msgs').msg;
const sens=require('../script/ycam1h.js');
const std_msgs=ros.require('std_msgs').msg;
const std_srvs=ros.require('std_srvs').srv;
const rovi_srvs = ros.require('rovi').srv;

let perf=ros.Time;
perf.msec=function(){
	let t=this.now();
	return t.secs*1e3+t.nsecs*1e-6;
}

ros.initNode('/test9').then((rosNode)=>{
	const pub_L=rosNode.advertise('/cam_L/image', sensor_msgs.Image);
	const pub_ct=rosNode.advertise('/cam_L/interval',std_msgs.Float32);
	const cli_remap=rosNode.serviceClient('/cam_L/remap/do',rovi_srvs.ImageFilter,{persist:true});
	rosNode.waitForService(cli_remap.getService(),2000)
	.then((available)=>{
		if(!available){
			ros.log.info('Service not available');
			return;
		}
		return rosNode.getParam('/cam_L/ID');
	})
	.then((id)=>{
		let ev=sens.start(rosNode,id);
		return Promise.resolve(ev);
	})
	.then((ev)=>{
		let remap_req=new rovi_srvs.ImageFilter.Request();
		let msg_ct=new std_msgs.Float32();
		let ct_start=perf.msec();
		ev.on('cam_L',function(img){
			remap_req.img=img;
			ct_start=perf.msec();
			cli_remap.call(remap_req)
			.then((res)=>{
				let ct=perf.msec();
				pub_L.publish(res.img);
				msg_ct.data=ct-ct_start;
				pub_ct.publish(msg_ct);
			});
		});
	});
	const svc1=rosNode.advertiseService('/cam_L/extern',std_srvs.SetBool, (req,res)=>{
		sens.set({'TriggerMode':req.data? 'On':'Off','AcquisitionFrameRate':10.0});
		return res.success=true;
	});
});
