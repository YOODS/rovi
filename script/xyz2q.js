#!/usr/bin/env node

const ros=require('rosnodejs');
const geometry_msgs=ros.require('geometry_msgs').msg;
const std_msgs=ros.require('std_msgs').msg;
const std_srvs=ros.require('std_srvs').srv;
const rovi_srvs=ros.require('rovi').srv;

function XYZToQuaternion(e){
	let q=e;
	let k=Math.PI/180*0.5;
	let cx=Math.cos(e[3]*k);
	let cy=Math.cos(e[4]*k);
	let cz=Math.cos(e[5]*k);
	let sx=Math.sin(e[3]*k);
	let sy=Math.sin(e[4]*k);
	let sz=Math.sin(e[5]*k);
	q[3]=cy*cz*sx-cx*sy*sz;
	q[4]=cy*sx*sz+cx*cz*sy;
	q[5]=cx*cy*sz-cz*sx*sy;
	q[6]=sx*sy*sz+cx*cy*cz;
	return q;
}

setImmediate(async function(){
	if(process.argv.length<3){
		ros.log.error('Missing target service name');
		return;
	}
	const rosNode=await ros.initNode('/xyz2q');
	let cl_pose=rosNode.serviceClient(process.argv[2],rovi_srvs.SetPose,{persist:true});
	if(! rosNode.waitForService(cl_pose.getService(),1000)){
		ros.log.error(process.arg[2]+' not available');
		return;
	}
	const svc=rosNode.advertiseService('/xyz2q',rovi_srvs.dialog,async function(rq,rs){
		let line=rq.hello;
		let strs=line.split(' ');
		let vals=[];
		strs.forEach(function(s){
			vals.push(parseFloat(s));
		});
		let qt=XYZToQuaternion(vals);
		let req=new rovi_srvs.SetPose.Request();
		req.pose.position.x=qt[0];
		req.pose.position.y=qt[1];
		req.pose.position.z=qt[2];
		req.pose.orientation.x=qt[3];
		req.pose.orientation.y=qt[4];
		req.pose.orientation.z=qt[5];
		req.pose.orientation.w=qt[6];
//console.log('XYZ-Q:'+JSON.stringify(req.pose));
		let res=await cl_pose.call(req);
		return true;
	});
});
