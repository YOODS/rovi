#!/usr/bin/env node

const NSthis='/rovi/pshift_genpc';
const NScamL='/rovi/left';
const NScamR='/rovi/right';
const NSlive='/rovi/live';
const NSrovi='/rovi';
const ros=require('rosnodejs');
const sensor_msgs=ros.require('sensor_msgs').msg;
const sensName=process.argv.length<3? 'ycam1s':process.argv[2];
const sens=require('./'+sensName+'.js')
const std_msgs=ros.require('std_msgs').msg;
const std_srvs=ros.require('std_srvs').srv;
const rovi_srvs = ros.require('rovi').srv;
const EventEmitter=require('events').EventEmitter;

class Streamer extends EventEmitter{
	constructor(rosNode,NS,camEv,ename){
		super();
		this.node=rosNode;
		this.NS=NS;
		this.pub_raw=rosNode.advertise(NS+'/image_raw', sensor_msgs.Image);
		this.pub_rect=rosNode.advertise(NS+'/image_rect', sensor_msgs.Image);
		this.pub_vue=rosNode.advertise(NS+'/view', sensor_msgs.Image);
		this.pub_cinfo=rosNode.advertise(NS+'/camera_info', sensor_msgs.CameraInfo);
		this.cl_remap=rosNode.serviceClient(NS+'/remap',rovi_srvs.ImageFilter,{persist:true});
		if(! rosNode.waitForService(this.cl_remap.getService(),1000)){
			ros.log.warn(NS+' remap not available');
			this.cl_remap=null;
		}
		this.cinfo=new sensor_msgs.CameraInfo();
		let ce=this;
		camEv.on(ename,async function(img){
			ce.pub_raw.publish(img);
			let req=new rovi_srvs.ImageFilter.Request();
			req.img=img;
			let res=this.cl_remap==null? req:await ce.cl_remap.call(req);
			if(ce.listenerCount('capt')>0) ce.emit('capt',res.img);
			else ce.pub_rect.publish(res.img);
			ce.cinfo.header=req.img.header;
			ce.pub_cinfo.publish(ce.cinfo);
		});
	}
	async reload(){
		Object.assign(this.cinfo,await this.node.getParam(this.NS+'/remap'));
	}
	view(n){
		try{
			if(n<this.imgs.length) this.pub_vue.publish(this.imgs[n]);
		}
		catch(err){
			ros.log.warn('No image captured:'+err);
		}
	}
	capture(n){
		let ce=this;
		return new Promise(function(resolve){
			let capt=[];
			ce.on('capt',function(img){
				capt.push(img);
				if (capt.length==n) {
					ros.log.info('capture resolve.');
					resolve(capt);
					ce.abort();
				}
			});
		});
	}
	abort(){//abort capture
		this.removeAllListeners('capt');
	}
}

setImmediate(async function(){
	const rosNode=await ros.initNode(NSthis);
	const sensEv=sens.open(rosNode,NSrovi);
	const capt_L=new Streamer(rosNode,NScamL,sensEv,'left');
	const capt_R=new Streamer(rosNode,NScamR,sensEv,'right');
	let vue_N=0;
	sensEv.on('wake',async function(s){
		capt_L.reload();
		capt_R.reload();
		let param=await rosNode.getParam(NSlive+'/camera');
		sens.cset(param);
	});
//---------Definition of services
	const svc_do=rosNode.advertiseService(NSthis,std_srvs.Trigger,(req,res)=>{
		if(!sens.normal){
			ros.log.warn('YCAM not ready');
			res.message='Sensor not ready';
			res.success=false;
			return true;
		}
		return new Promise(async (resolve)=>{
			let param_P=await rosNode.getParam(NSthis+'/projector');
			let param_V=await rosNode.getParam(NSlive+'/camera');
			let param_C=await rosNode.getParam(NSthis+'/camera');
			for(let key in param_V) if(!param_C.hasOwnProperty(key)) delete param_V[key];
			let wdt=setTimeout(function(){//<--------watch dog
				resolve(false);
				capt_L.abort();
				capt_R.abort();
				sens.cset(param_V);
				sens.cset({'TriggerMode':'Off'});
			},param_P.Interval*20);
			sens.cset({'TriggerMode':'On'});
			sens.cset(param_C);
			sens.pset('x'+param_P.ExposureTime);
			sens.pset((sensName.startsWith('ycam3')? 'o':'p')+param_P.Interval);
			let val=param_P.Intencity<256? param_P.Intencity:255;
			val=val.toString(16);
			sens.pset('i'+val+val+val);
			sens.pset(sensName.startsWith('ycam3')? 'o2':'p2');//<--------projector sequence start
			let imgs=await Promise.all([capt_L.capture(13),capt_R.capture(13)]);
			clearTimeout(wdt);
			capt_L.imgs=imgs[0];
			capt_R.imgs=imgs[1];
			sens.cset(param_V);
			sens.cset({'TriggerMode':'Off'});
			res.message='scan compelete:'+imgs[0].length;
			res.success=true;
			capt_L.view(vue_N);
			capt_R.view(vue_N);
			resolve(true);
		});
	});
	const svc_parse=rosNode.advertiseService(NSthis+'/parse',rovi_srvs.dialog,(req,res)=>{
		let cmd=req.hello;
		let lbk=cmd.indexOf('{');
		let obj={};
		if(lbk>0){
			cmd=req.hello.substring(0,lbk).trim();
			try {
				obj=JSON.parse(req.hello.substring(lbk));
			}
			catch(err){
				//ignore
			}
		}
		let cmds=cmd.split(' ');
		if(cmds.length>1) cmd=cmds.shift();
		switch(cmd){
		case 'cset':
			sens.cset(obj);
			for(let key in obj){
				rosNode.setParam(NSlive+'/camera/'+key,obj[key]);
			}
			return Promise.resolve(true);
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
				capt_L.view(vue_N);
				capt_R.view(vue_N);
				resolve(true);
			});
		}
	});
});
