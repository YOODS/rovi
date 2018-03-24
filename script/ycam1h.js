const Net=require('net');
const EventEmitter=require('events').EventEmitter;
const Rosrun=require('./rosrun.js');
const Notifier=new EventEmitter;

const ros=require('rosnodejs');
const sensor_msgs=ros.require('sensor_msgs').msg;
const std_msgs=ros.require('std_msgs').msg;
const std_srvs=ros.require('std_srvs').srv;
const dyn_msgs=ros.require('dynamic_reconfigure').msg;
const dyn_srvs=ros.require('dynamic_reconfigure').srv;
const rovi_srvs=ros.require('rovi').srv;

let run_l;  //should be rosrun.js camnode left
let run_r;  //should be rosrun.js camnode right
let run_p;  //should be openYPJ:Socket
let rosNode;
let camera_l='/camera/';
let camera_r='/camera/';

function setDblConf(req,key,val){
	var param=new dyn_msgs.DoubleParameter();
	param.name=key;
	param.value=val;
	req.config.doubles.push(param);
}
var ycam={
	cset:async function(obj){
		let request=new dyn_srvs.Reconfigure.Request();
		let key;
		if(obj.hasOwnProperty(key='TriggerMode')){
			var param=new dyn_msgs.StrParameter();
			param.name=key;
			param.value=obj[key];
			request.config.strs.push(param);
		}
		if(obj.hasOwnProperty(key='AcquisitionFrameRate')) setDblConf(request,key,obj[key]);
		if(obj.hasOwnProperty(key='Gain')) setDblConf(request,key,obj[key]);
		if(obj.hasOwnProperty(key='ExposureTimeAbs')) setDblConf(request,key,obj[key]);
		let res_l=await run_l.dynparam_set.call(request);
//		let res_r=await run_r.dynparam_set.call(request);
		return true;
	},
	cget:async function(ary){
		let ret={};
		for(let i=0;i<ary.length;i++){
			let key=ary[i];
			ret[key]=await rosNode.getParam(camera_l+key);
		}
		return ret;
	},
	pset:function(str){
		run_p.write(str+'\n');
	},
	stat:function(){
		return run_l.running && !run_p.destroyed;
	},
	open:function(nh,nsl,idl,nsr,idr,url,port){
		rosNode=nh;
		camera_l=nsl+'/camera/';
		run_l=Rosrun.run('camera_aravis camnode '+idl,nsl);
		run_l.on('start',function(){
			openCamera(run_l,camera_l,'cam_l');
		});
		camera_r=nsr+'/camera/';
//		run_r=Rosrun.run('camera_aravis camnode '+idr,nsr);
//		run_r.on('start',function(){
//			openCamera(run_r,camera_r,'cam_r');
//		});
		run_p=openYPJ(port,url);
		return Notifier;
	}
}

async function openCamera(rosrun,ns,evname){
	let sub=rosNode.subscribe(ns+'image_raw',sensor_msgs.Image,(src)=>{
		Notifier.emit(evname,src);
	});
	var cset=rosNode.serviceClient(ns+'set_parameters',dyn_srvs.Reconfigure,{persist:true});
	if(! await rosNode.waitForService(cset.getService(),2000)){
		ros.log.error('Service not available');
		return;
	}
	rosrun.dynparam_set=cset;
}

function openYPJ(port,url,sock){
	if(arguments.length<3) sock=new Net.Socket();
	sock.on('connect',function(){
		ros.log.info('YPJ connected');
	});
	sock.on('error',function(){
		ros.log.error('YPJ error');
	});
	sock.on('close',function(){
		ros.log.info('YPJ closed');
		setTimeout(function(){
			sock.connect(port,url);
		},3000);
	});
	sock.on('data',function(data){
		Notifier.emit('pout',data);
	});
	sock.connect(port,url);
	return sock;
}

module.exports=ycam;
