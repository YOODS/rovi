#!/usr/bin/env node

const NSthis='/rovi/pshift_genpc';
const NScamL='/rovi/left';
const NScamR='/rovi/right';
const NSlive='/rovi/live';
const NSrovi='/rovi';
const ros=require('rosnodejs');
const sensor_msgs=ros.require('sensor_msgs').msg;
const geometry_msgs=ros.require('geometry_msgs').msg;
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
		}
	});
});
