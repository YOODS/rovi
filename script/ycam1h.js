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

diffTime=function(t1,t0){
	let dt=ros.Time.now();
	dt.secs=t1.secs-t0.secs;
	dt.nsecs=t1.nsecs-t0.nsecs;
	return ros.Time.toSeconds(dt);
}

var ycam={
	cset:async function(obj){
		let request=new dyn_srvs.Reconfigure.Request();
		for(let key in obj){
			let val=obj[key];
			if(typeof(val)=='string'){
				let param=new dyn_msgs.StrParameter();
				param.name=key;
				param.value=val;
				request.config.strs.push(param);
console.log('ycam.cset as string:'+key+'='+val);
			}
			else{
				let param=new dyn_msgs.DoubleParameter();
				param.name=key;
				param.value=val;
				request.config.doubles.push(param);
console.log('ycam.cset as double:'+key+'='+val);
			}
		}
		let res_l=await run_l.dynparam_set.call(request);
		let res_r=await run_r.dynparam_set.call(request);
		return true;
	},
	pset:function(str){
		run_p.write(str+'\n');
		run_p.setNoDelay(true);
	},
	normal:false,
	stat:function(){
		return {'left':run_l.running, 'right':run_r.running, 'projector':!run_p.destroyed};
	},
	scan:function(){
		let s;
		try{
			s=this.stat();
		}
		catch(err){
			Notifier.emit('stat',this.normal=false);
			setTimeout(function(){ ycam.stat();},1000);
			return;
		}
		let f=true;
		for(let key in s) f=f && s[key];
		if(f==undefined) f=false;
		Notifier.emit('stat',this.normal=f);
		setTimeout(function(){ ycam.scan();},1000);
	},
	open:function(nh,nsl,idl,nsr,idr,url,port){
		rosNode=nh;
		camera_l=nsl+'/camera/';
		run_l=Rosrun.run('camera_aravis camnode '+idl,nsl);
		run_l.on('start',function(){
			openCamera(run_l,camera_l,'left');
		});
		camera_r=nsr+'/camera/';
		run_r=Rosrun.run('camera_aravis camnode '+idr,nsr);
		run_r.on('start',function(){
			openCamera(run_r,camera_r,'right');
		});
		this.scan();
		setTimeout(function(){
			run_p=openYPJ(port,url);
		},5000);
		return Notifier;
	}
}

async function openCamera(rosrun,ns,evname){
	let sub=rosNode.subscribe(ns+'image_raw',sensor_msgs.Image,(src)=>{
		if(diffTime(src.header.stamp,rosrun.timestamp)>0.01){
			Notifier.emit(evname,src);
			rosrun.timestamp=src.header.stamp;
		}
	});
	var cset=rosNode.serviceClient(ns+'set_parameters',dyn_srvs.Reconfigure,{persist:true});
	if(! await rosNode.waitForService(cset.getService(),2000)){
		ros.log.error('Service not available');
		return;
	}
	rosrun.dynparam_set=cset;
	rosrun.timestamp=ros.Time.now();
}

function openYPJ(port,url,sock){
	if(arguments.length<3) sock=new Net.Socket();
	sock.on('connect',function(){
		ros.log.info('***YPJ connected***');
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
	sock.on('drain',function(data){
		console.log('YPJ drain');
	});
	sock.connect(port,url);
	return sock;
}

module.exports=ycam;
