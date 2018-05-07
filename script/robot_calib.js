#!/usr/bin/env node

const ros=require('rosnodejs');
const sensor_msgs=ros.require('sensor_msgs').msg;
const geometry_msgs=ros.require('geometry_msgs').msg;
const std_msgs=ros.require('std_msgs').msg;
const std_srvs=ros.require('std_srvs').srv;
const rovi_srvs=ros.require('rovi').srv;
const EventEmitter=require('events').EventEmitter;

setImmediate(async function(){
	const rosNode=await ros.initNode('robot_calib');
	const evs=new EventEmitter();
	let pos_grid=new geometry_msgs.Pose();
	let cl_grid=rosNode.serviceClient('/gridboard',rovi_srvs.GetGrid,{persist:true});
	if(! rosNode.waitForService(cl_grid.getService(),1000)){
		ros.log.error('grid_node not available');
		return;
	}
	const svc1=rosNode.advertiseService('/robot_calib/pose',rovi_srvs.SetPose,(req,res)=>{
		ros.log.info('robot_pose:'+JSON.stringify(req.pose));
		return new Promise(function(resolve){
			evs.once('pose',function(pose){
				ros.log.info('object_pose:'+JSON.stringify(pose));
				resolve(true);
//				evs.removeAllListeners('pose');
			});
		});
	});
	let pub_grid=rosNode.advertise('/robot_calib/image', sensor_msgs.Image);
	let sub=rosNode.subscribe('/left/image_rect',sensor_msgs.Image,async (src)=>{
		let req=new rovi_srvs.GetGrid.Request();
		req.img=src;
		let res=await cl_grid.call(req);
		pub_grid.publish(res.img);
		evs.emit('pose',res.pose);
	});
});
