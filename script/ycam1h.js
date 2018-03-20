const EventEmitter=require('events').EventEmitter;

const ros=require('rosnodejs');
const sensor_msgs=ros.require('sensor_msgs').msg;
const std_msgs=ros.require('std_msgs').msg;
const std_srvs=ros.require('std_srvs').srv;
const dyn_msgs=ros.require('dynamic_reconfigure').msg;
const dyn_srvs=ros.require('dynamic_reconfigure').srv;
const rovi_srvs=ros.require('rovi').srv;

let run_l;  //rosrun.js camnode left
let run_R;  //rosrun.js camnode right
let run_P;  //rosrun.js projector
let rosNode;
let camera_l='/cam_L/camera/';
let camera_r='/cam_R/camera/';
let projector='/projector/';
let notifier;

function setDblConf(req,key,val){
	var param=new dyn_msgs.DoubleParameter();
	param.name=key;
	param.value=val;
	req.config.doubles.push(param);
}
var ycam={
	set:async function(obj){
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
		let resp=await run_l.setparam.call(request);
		return true;
	},
	get:async function(ary){
		let ret={};
		for(let i=0;i<ary.length;i++){
			let key=ary[i];
			ret[key]=await rosNode.getParam(camera_l+key);
		}
		return ret;
	},
	pset:async function(obj){},
	stat:function(){
		return run_l.running;
	},
	open:function(nh,idl){
		rosNode=nh;
		run_l=require('./rosrun.js').run('camera_aravis camnode '+idl,'/cam_L');
		run_l.on('start',function(){
			openCamera(run_l,camera_l,'cam_L');
		});
//		run_r=require('./rosrun.js').run('camera_aravis camnode '+idr,'/cam_R');
//		run_r.on('start',function(){
//			openCamera(run_r,camera_r,'cam_R');
//		});
//		run_p=require('./rosrun.js').run('rovi projector.js');
		return notifier=new EventEmitter();
	}
}

async function openCamera(rosrun,ns,evname){
	let sub=rosNode.subscribe(ns+'image_raw',sensor_msgs.Image,(src)=>{
		notifier.emit(evname,src);
	});
	var cset=rosNode.serviceClient(ns+'set_parameters',dyn_srvs.Reconfigure,{persist:true});
	if(! await rosNode.waitForService(cset.getService(),2000)){
		ros.log.error('Service not available');
		return;
	}
	rosrun.setparam=cset;
}

module.exports=ycam;
