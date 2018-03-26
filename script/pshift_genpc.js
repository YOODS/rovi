#!/usr/bin/env node

const NS='/pshift_genpc';
const NScamL='/cam_l';
const NScamR='/cam_r';
const ros=require('rosnodejs');
const sensor_msgs=ros.require('sensor_msgs').msg;
const sens=require('../script/ycam1h.js');
const std_msgs=ros.require('std_msgs').msg;
const std_srvs=ros.require('std_srvs').srv;
const rovi_srvs = ros.require('rovi').srv;

ros.Time.diff=function(t0){
	let t1=ros.Time.now();
	t1.secs-=t0.secs;
	t1.nsecs-=t0.nsecs;
	return ros.Time.toSeconds(t1);
}

function sensCheck(pub){
	let f=new std_msgs.Bool();
	let s=sens.stat();
	f.data=true;
	for(let key in s) f.data=f.data && s[key];
	pub.publish(f);
	setTimeout(function(){ sensCheck(pub);},1000);
}
function viewOut(n,pubL,capL,pubR,capR){
	if(n<capL.length){
ros.log.info('capture L image:'+n);
		pubL.publish(capL[n]);
	}
	if(n<capR.length) pubR.publish(capR[n]);
}

setImmediate(async function(){
	const rosNode=await ros.initNode(NS);
	const pub_tat=rosNode.advertise(NS+'/tat', std_msgs.Float32);
	const pub_stat=rosNode.advertise(NS+'/stat', std_msgs.Bool);
	let vue_N=0;

	const pub_L=rosNode.advertise(NScamL+'/image', sensor_msgs.Image);
	const vue_L=rosNode.advertise(NScamL+'/view', sensor_msgs.Image);
	const remap_L=rosNode.serviceClient(NScamL+'/remap',rovi_srvs.ImageFilter,{persist:true});
	if(! await rosNode.waitForService(remap_L.getService(),2000)){
		ros.log.info('remap_L service not available');
		return;
	}
	const pub_R=rosNode.advertise(NScamR+'/image', sensor_msgs.Image);
	const vue_R=rosNode.advertise(NScamR+'/view', sensor_msgs.Image);
	const remap_R=rosNode.serviceClient(NScamR+'/remap',rovi_srvs.ImageFilter,{persist:true});
	if(! await rosNode.waitForService(remap_R.getService(),2000)){
		ros.log.info('remap_R service not available');
		return;
	}
	let param_L=await rosNode.getParam(NScamL+'/camera');
	let param_R=await rosNode.getParam(NScamR+'/camera');
	let param_P=await rosNode.getParam(NS+'/projector');
	let param_C=await rosNode.getParam(NS+'/camera');

	const sensEv=sens.open(rosNode,NScamL,param_L.ID,NScamR,param_R.ID,param_P.Url,param_P.Port);//<--------open ycam
	let hook_L=null;
	let hook_R=null;
	sensEv.on('cam_l',async function(img){//<--------a left eye image comes up
		let req=new rovi_srvs.ImageFilter.Request();
		req.img=img;
		let res=await remap_L.call(req);
		if(hook_L==null){
			pub_L.publish(res.img);
		}
		else hook_L(res.img);
	});
	sensEv.on('cam_r',async function(img){//<--------a right eye image comes up
		let req=new rovi_srvs.ImageFilter.Request();
		req.img=img;
		let res=await remap_R.call(req);
		if(hook_R==null){
			pub_R.publish(res.img);
		}
		else hook_R(res.img);
	});
	sensEv.on('pout',function(str){//<--------cout from YPJ
		console.log('projector :'+str);
	});
	sensCheck(pub_stat);//<--------start checking devices, and output to the topic "stat"
	let capt_L=[];
	let capt_R=[];
	const svc_do=rosNode.advertiseService(NS,std_srvs.Trigger,(req,res)=>{//<--------generate PCL
		return new Promise(async (resolve)=>{
			sens.cset({'TriggerMode':'On'});
			param_C=await rosNode.getParam(NS+'/camera');
			sens.cset(param_C);
			param_L=await rosNode.getParam(NScamL+'/camera');
			for(let key in param_L) if(!param_C.hasOwnProperty(key)) delete param_L[key];
			param_P=await rosNode.getParam(NS+'/projector');
			let wdt=setTimeout(function(){//<--------watch dog
				resolve(false);
				hook_L=hook_R=null;
				sens.cset(Object.assign({'TriggerMode':'Off'},param_L));
			},2000);
			sens.pset('x'+param_P.ExposureTime);
			sens.pset('p'+param_P.Interval);
			let val=param_P.Intencity<256? param_P.Intencity:255;
			val=val.toString(16);
			sens.pset('p'+val+val+val);
			sens.pset('p2');//<--------projector sequence start
			let tat=new std_msgs.Float32();
			tat.data=ros.Time.now();
			let imgs=await Promise.all([
				new Promise((resolve)=>{
					capt_L=[];
					hook_L=function(img){
						capt_L.push(img);
						if(capt_L.length==13){
							resolve(capt_L);
							hook_L=null;
						}
					}
				}),
				new Promise((resolve)=>{
					capt_R=[];
					hook_R=function(img){
						capt_R.push(img);
						if(capt_R.length==13){
							resolve(capt_R);
							hook_R=null;
						}
					}
				})
			]);
			clearTimeout(wdt);
			sens.cset(Object.assign({'TriggerMode':'Off'},param_L));
			tat.data=ros.Time.diff(tat.data);
			pub_tat.publish(tat);
			res.answer='scan compelete:'+imgs[0].length;
			res.success=true;
			ros.log.info('capture completed');
			viewOut(vue_N,vue_L,capt_L,vue_R,capt_R);
			resolve(true);
		});
	});
	const svc_parse=rosNode.advertiseService(NS+'/parse',rovi_srvs.dialog,(req,res)=>{
		let cmd=req.hello;
		let lbk=cmd.indexOf('{');
		let obj={};
		if(lbk>0){
			cmd=req.hello.substring(0,lbk).trim();
			obj=JSON.parse(req.hello.substring(lbk));
		}
		let cmds=cmd.split(' ');
		if(cmds.length>1) cmd=cmds.shift();
		switch(cmd){
		case 'pset':
			sens.pset(cmds[0]);
			return Promise.resolve(true);
		case 'stat'://<--------sensor(maybe YCAM) status query
			return new Promise((resolve)=>{
				res.answer=JSON.stringify(sens.stat());
				resolve(true);
			});
		case 'view':
			return new Promise((resolve)=>{
				vue_N=parseInt(cmds[0]);
				viewOut(vue_N,vue_L,capt_L,vue_R,capt_R);
				resolve(true);
			});
		}
	});
});
