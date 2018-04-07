#!/usr/bin/env node

const net=require('net');
const shm=require('../shm-typed-array/index.js');

const ros=require('rosnodejs');
const sensor_msgs=ros.require('sensor_msgs').msg;
const std_msgs=ros.require('std_msgs').msg;
const std_srvs=ros.require('std_srvs').srv;
const rovi_srvs = ros.require('rovi').srv;
const EventEmitter=require('events').EventEmitter;
let ev=new EventEmitter();

let popen=require('child_process');
let shmem;
let imgLength;

ros.Time.diff=function(t0){
	let t1=ros.Time.now();
	t1.secs-=t0.secs;
	t1.nsecs-=t0.nsecs;
	return ros.Time.toSeconds(t1);
}

function creatImg(param){
	let img=new sensor_msgs.Image();
	img.width=640;
	img.height=480;
	img.step=640;
	img.encoding='mono8';
	for(let key in img){
		if(param.hasOwnProperty(key)){
			img[key]=param[key];
		}
	}
	imgLength=img.height*img.step;
	return img;
}

setImmediate(async function(){
	const rosNode=await ros.initNode('/test10');
	const pub_L=rosNode.advertise('/cam_l/image', sensor_msgs.Image);
	const pub_R=rosNode.advertise('/cam_r/image', sensor_msgs.Image);
	let img_L=creatImg({'width':1280,'height':1024,'step':1280,'encoding':'mono8'});
	let img_R=creatImg({'width':1280,'height':1024,'step':1280,'encoding':'mono8'});

	process.stdin.on('close', function(){
		ros.log.info('server-> client closed connection');
	});
	process.stdin.on('data',function(data){
		let attr;
		try{
			attr=JSON.parse(data);
		}
		catch(err){
			ros.log.warn('stdin:'+err);
			return;
		}
		if(attr.hasOwnProperty('capt')){
			let offset=attr.capt;
ros.log.info('shm capt:'+offset);
			if(offset==0){
				img_L.data=shmem.slice(0,imgLength);
				pub_L.publish(img_L);
			}
			else{
				img_R.data=shmem.slice(offset,offset+imgLength);
				pub_R.publish(img_R);
			}
		}
		else if(attr.hasOwnProperty('shm')){
			shmem=shm.get(attr.shm,'Uint8Array');
		}
	});
});
