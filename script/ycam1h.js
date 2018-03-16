const EventEmitter=require('events').EventEmitter;
let notif=new EventEmitter();
let run_l;//=rosrun
let run_r;//=rosrun

const ros=require('rosnodejs');
const sensor_msgs=ros.require('sensor_msgs').msg;
const std_msgs=ros.require('std_msgs').msg;
const std_srvs=ros.require('std_srvs').srv;
const dyn_msgs=ros.require('dynamic_reconfigure').msg;
const dyn_srvs=ros.require('dynamic_reconfigure').srv;
const rovi_srvs=ros.require('rovi').srv;

function setDblConf(req,key,val){
	var param=new dyn_msgs.DoubleParameter();
	param.name=key;
	param.value=val;
	req.config.doubles.push(param);
}
var sens={
	set:function(obj){
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
		run_l.target.call(request).then(function(resp){
			return true;
		});
	},
	stat:function(){
		return run_l.running;
	},
	start:function(nh,id){
		run_l=require('./rosrun.js').run('camera_aravis camnode '+id,'/cam_L');
		run_l.on('start',function(){
			camera_start(nh,run_l,'/cam_L/camera/','cam_L');
		});
		return notif;
	}
}

function camera_start(nh,rosrun,ns,evname){
	let sub=nh.subscribe(ns+'image_raw',sensor_msgs.Image,(src)=>{
		notif.emit(evname,src);
	});
	var cli=nh.serviceClient(ns+'set_parameters',dyn_srvs.Reconfigure,{persist:true});
	nh.waitForService(cli.getService(),2000).then(function(available){
		if(!available){
			ros.log.info('Service not available');
			return;
		}
		rosrun.target=cli;
	});
}

module.exports=sens;
