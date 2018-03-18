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

async function main(){
	const rosNode=await ros.initNode('/test9');
	const pub_ct=rosNode.advertise('/test9/data',std_msgs.Float32);
	const msg_ct=new std_msgs.Float32();

	const pub_L=rosNode.advertise('/cam_L/image', sensor_msgs.Image);
	const remap_L=rosNode.serviceClient('/cam_L/remap/do',rovi_srvs.ImageFilter,{persist:true});
	if(! await rosNode.waitForService(remap_L.getService(),2000)){
		ros.log.info('remap_L service not available');
		return;
	}
//	if(! await rosNode.waitForService(remap_R.getService(),2000)){
//		ros.log.info('remap_R service not available');
//		return;
//	}
	const id_L=await rosNode.getParam('/cam_L/ID');
//	const id_R=await rosNode.getParam('/cam_R/ID');
	const ev=sens.start(rosNode,id_L);
	ev.on('cam_L',async function(img){
		let req=new rovi_srvs.ImageFilter.Request();
		req.img=img;
		let ct_start=perf.msec();
		let res=await remap_L.call(req);
		let ct=perf.msec();
		pub_L.publish(res.img);
		msg_ct.data=ct-ct_start;
		pub_ct.publish(msg_ct);
	});
	const svc_parse=rosNode.advertiseService('/test9/parse',rovi_srvs.dialog,(req,res)=>{
		let cmd=req.hello;
		let lbk=cmd.indexOf('{');
		let obj={};
		if(lbk>0){
			cmd=req.hello.substring(0,lbk).trim();
			obj=JSON.parse(req.hello.substring(lbk));
		}
		ros.log.info('parsed:'+cmd+' arg:'+JSON.stringify(obj));
		switch(cmd){
		case 'stat':
			return new Promise((resolve)=>{
				res.answer='{"camera":'+sens.stat()+'}';
				resolve(true);
			});
		case 'ext':  //external(Line1 or Software) trigger
			sens.set({'TriggerMode':'On','AcquisitionFrameRate':10.0});
			return Promise.resolve(true);
		case 'int':  //internal(hardware) trigger
			sens.set({'TriggerMode':'Off','AcquisitionFrameRate':10.0});
			return Promise.resolve(true);
		}
	});
}


main();
