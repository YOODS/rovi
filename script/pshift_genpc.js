#!/usr/bin/env node

const NS='/rovi/pshift_genpc';
const NScamL='/rovi/cam_l';
const NScamR='/rovi/cam_r';
const NSlive='/rovi/live';
const NSrovi='/rovi';
const ros=require('rosnodejs');
const sensor_msgs=ros.require('sensor_msgs').msg;
const sens=require('../script/ycam1s.js');
const std_msgs=ros.require('std_msgs').msg;
const std_srvs=ros.require('std_srvs').srv;
const rovi_srvs = ros.require('rovi').srv;
const EventEmitter=require('events').EventEmitter;

const imgdbg = false;

ros.Time.diff=function(t0){
	let t1=ros.Time.now();
	t1.secs-=t0.secs;
	t1.nsecs-=t0.nsecs;
	return ros.Time.toSeconds(t1);
}

function copyImg(src) {
	let img = new sensor_msgs.Image();
	img.header.seq = src.header.seq;
	img.header.stamp = src.header.stamp;
	img.header.frame_id = src.header.frame_id;
	img.width = src.width;
	img.height = src.height;
	img.step = src.step;
	img.encoding = src.encoding;
	img.data = src.data.slice(0, img.width * img.height);
	return img;
}

let sensStat=false;
function sensCheck(pub){
	let f=new std_msgs.Bool();
	let s=sens.stat();
	f.data=true;
	for(let key in s) f.data=f.data && s[key];
	pub.publish(f);
	sensStat=f.data;
	setTimeout(function(){ sensCheck(pub);},1000);
}
function viewOut(n,pubL,capL,pubR,capR){
	try{
		if (!imgdbg) {
			if(n<capL.length) pubL.publish(capL[n]);
			if(n<capR.length) pubR.publish(capR[n]);
		}
		else {
			if(n<capL.length) {
				pubL.publish(capL[n]);
				ros.log.warn("n(" + n + "<capL.length(" + capL.length + "). published seq=" + capL[n].header.seq);
				for (let li = 0; li < capL.length; li++) {
					ros.log.warn("get capL[" + li + "].seq=" + capL[li].header.seq);
				}
			}
			if(n<capR.length) {
				pubR.publish(capR[n]);
				ros.log.warn("n(" + n + "<capR.length(" + capR.length + "). published seq=" + capR[n].header.seq);
				for (let ri = 0; ri < capR.length; ri++) {
					ros.log.warn("get capR[" + ri + "].seq=" + capR[ri].header.seq);
				}
			}
		}
	}
	catch(err){
		ros.log.warn('No image captured:'+err);
	}
}

setImmediate(async function(){
	const rosNode=await ros.initNode(NS);
	const pub_tat=rosNode.advertise(NS+'/tat', std_msgs.Float32);
	const pub_stat=rosNode.advertise(NS+'/stat', std_msgs.Bool);
	let vue_N=0;

	const raw_L=rosNode.advertise(NScamL+'/image_raw', sensor_msgs.Image);
	const rect_L=rosNode.advertise(NScamL+'/image_rect', sensor_msgs.Image);
	const vue_L=rosNode.advertise(NScamL+'/view', sensor_msgs.Image);
	const remap_L=rosNode.serviceClient(NScamL+'/remap',rovi_srvs.ImageFilter,{persist:true});
	if(! await rosNode.waitForService(remap_L.getService(),2000)){
		ros.log.error('remap_L service not available');
		return;
	}
	const raw_R=rosNode.advertise(NScamR+'/image_raw', sensor_msgs.Image);
	const rect_R=rosNode.advertise(NScamR+'/image_rect', sensor_msgs.Image);
	const vue_R=rosNode.advertise(NScamR+'/view', sensor_msgs.Image);
	const remap_R=rosNode.serviceClient(NScamR+'/remap',rovi_srvs.ImageFilter,{persist:true});
	if(! await rosNode.waitForService(remap_R.getService(),2000)){
		ros.log.error('remap_R service not available');
		return;
	}
	const pub_pc=rosNode.advertise(NSrovi+'/pc', sensor_msgs.PointCloud);
	const genpc=rosNode.serviceClient(NSrovi+'/genpc',rovi_srvs.GenPC,{persist:true});
	if(! await rosNode.waitForService(genpc.getService(),2000)){
		ros.log.error('genpc service not available');
		return;
	}
	let param_L=await rosNode.getParam(NScamL+'/camera');
	let param_R=await rosNode.getParam(NScamR+'/camera');
	let param_P=await rosNode.getParam(NS+'/projector');
	let param_C=await rosNode.getParam(NS+'/camera');
	let param_V=await rosNode.getParam(NSlive+'/camera');

	for(let key in param_C) console.log(NS+'/camera/' + key + "=" + param_C[key]);
	for(let key in param_L) console.log(NScamL+'/camera/' + key + "=" + param_L[key]);
	for(let key in param_R) console.log(NScamR+'/camera/' + key + "=" + param_R[key]);
	for(let key in param_V) console.log(NSlive+'/camera/' + key + "=" + param_V[key]);

	const sensEv=sens.open(param_L.ID,param_R.ID,param_P.Url,param_P.Port,param_V);//<--------open ycam
	const sensHook=new EventEmitter();
	sensEv.on('cam_l',async function(img){//<--------a left eye image comes up
//ros.log.warn('capturing img_L');
		if (imgdbg) {
			ros.log.warn("from ycam1s cam_l seq=" + img.header.seq);
		}
		raw_L.publish(img);
		let req=new rovi_srvs.ImageFilter.Request();
		req.img=img;
		let res=await remap_L.call(req);
ros.log.warn('cam_l/image published. seq=' + img.header.seq);
		// for raw img genpc, replace res.img with copyImg(req.img)
		if(sensHook.listenerCount('cam_l')>0) sensHook.emit('cam_l',res.img);
/*
		if(sensHook.listenerCount('cam_l')>0) {
//			ros.log.warn("cam_l listener count(" + sensHook.listenerCount('cam_l') + ")> 0. emit seq=" + req.img.header.seq);
			sensHook.emit('cam_l',copyImg(req.img));
		}
*/
		else rect_L.publish(res.img);
	});
	sensEv.on('cam_r',async function(img){//<--------a right eye image comes up
//ros.log.warn('capturing img_R');
		if (imgdbg) {
			ros.log.warn("from ycam1s cam_r seq=" + img.header.seq);
		}
		raw_R.publish(img);
		let req=new rovi_srvs.ImageFilter.Request();
		req.img=img;
		let res=await remap_R.call(req);
ros.log.warn('cam_r/image published. seq=' + img.header.seq);
		// for raw img genpc, replace res.img with copyImg(req.img)
		if(sensHook.listenerCount('cam_r')>0) sensHook.emit('cam_r',res.img);
/*
		if(sensHook.listenerCount('cam_r')>0) {
//			ros.log.warn("cam_r listener count(" + sensHook.listenerCount('cam_r') + ")> 0. emit seq=" + req.img.header.seq);
			sensHook.emit('cam_r',copyImg(req.img));
		}
*/
		else rect_R.publish(res.img);
	});
	sensCheck(pub_stat);//<--------start checking devices, and output to the topic "stat"

//---------Definition of services
	let capt_L;//<--------captured images of the left camera
	let capt_R;//<--------same as right
	const svc_do=rosNode.advertiseService(NS,std_srvs.Trigger,(req,res)=>{//<--------generate PCL
ros.log.warn('pshift_genpc called!');
		if(!sensStat){
			ros.log.warn('YCAM not ready');
			return false;
		}
		return new Promise(async (resolve)=>{
			let wdt=setTimeout(function(){//<--------watch dog
				resolve(false);
				sensHook.removeAllListeners();
				sens.cset(Object.assign({'TriggerMode':'Off'},param_V));
ros.log.warn('in setTimeout');
			},param_P.Interval*20);
ros.log.warn('before cset TriggerMode:On');
			sens.cset({'TriggerMode':'On'});
ros.log.warn('after  cset TriggerMode:On');
			param_C=await rosNode.getParam(NS+'/camera');
			sens.cset(param_C);
			param_V=await rosNode.getParam(NSlive+'/camera');
			for(let key in param_V) if(!param_C.hasOwnProperty(key)) delete param_V[key];
			param_P=await rosNode.getParam(NS+'/projector');

ros.log.warn('now await setTimeout1');
await setTimeout(async function() {
ros.log.warn("setTimeout1 function start");
/*
			sens.pset('x'+param_P.ExposureTime);
			sens.pset('p'+param_P.Interval);
			let val=param_P.Intencity<256? param_P.Intencity:255;
			val=val.toString(16);
			sens.pset('i'+val+val+val);
*/
ros.log.warn('before pset p2');
			sens.pset('p2');//<--------projector sequence start
ros.log.warn('after  pset p2');
ros.log.warn("setTimeout1 function end");
			}, 100); // これはcsetが実際に反映されるのを待つ時間も兼ねる(ライブの残りカスを捨てるのも)

ros.log.warn('now await setTimeout2');
await setTimeout(async function() {
ros.log.warn("setTimeout2 function start");
			let imgs=await Promise.all([
				new Promise((resolve)=>{
					let capt=[];
					ros.log.warn("before sensHook.on('cam_l')");
					sensHook.on('cam_l',function(img){
ros.log.warn('capturing img_L:'+capt.length+" seq="+img.header.seq);
						if (imgdbg) {
							ros.log.warn("capt ycam1s cam_l seq=" + img.header.seq + " ... " + capt.length);
						}
						if (capt.length <= 11) {
							capt.push(img);
						}
						else if (capt.length == 12) {
							capt.push(img);
							ros.log.warn('now 13 img_Ls. resolve.');
							resolve(capt);
						}
						else { // already capt.length >= 13.
							ros.log.warn('already 13 img_Ls. ignore this img.');
						}
					});
					ros.log.warn("after  sensHook.on('cam_l'");
				}),
				new Promise((resolve)=>{
					let capt=[];
//					resolve(capt);
					ros.log.warn("before sensHook.on('cam_r')");
					sensHook.on('cam_r',function(img){
ros.log.warn('capturing img_R:'+capt.length+" seq="+img.header.seq);
						if (imgdbg) {
							ros.log.warn("capt ycam1s cam_r seq=" + img.header.seq + " ... " + capt.length);
						}
						if (capt.length <= 11) {
							capt.push(img);
						}
						else if (capt.length == 12) {
							capt.push(img);
							ros.log.warn('now 13 img_Rs. resolve.');
							resolve(capt);
						}
						else { // already capt.length >= 13.
							ros.log.warn('already 13 img_Rs. ignore this img.');
						}
					});
					ros.log.warn("after  sensHook.on('cam_r'");
				})
			]);
ros.log.warn('after await Promise.all');
			ros.log.warn("before sensHook.removeAllListeners()");
			sensHook.removeAllListeners();
			ros.log.warn("after  sensHook.removeAllListeners()");
			clearTimeout(wdt);
			capt_L=imgs[0];
			capt_R=imgs[1];
ros.log.warn('capt_L and capt_R set. capt_L.length=' + capt_L.length + ", capt_R.length=" + capt_R.length);

			for (let li = 0; li < capt_L.length; li++) {
				ros.log.warn("Set capt_L[" + li + "].seq=" + capt_L[li].header.seq);
			}
			for (let ri = 0; ri < capt_R.length; ri++) {
				ros.log.warn("Set capt_R[" + ri + "].seq=" + capt_R[ri].header.seq);
			}

			ros.log.warn("genpc CALL");
			let gpreq = new rovi_srvs.GenPC.Request();
			gpreq.imgL = capt_L;
			gpreq.imgR = capt_R;
			let gpres=await genpc.call(gpreq);
			pub_pc.publish(gpres.pc);
			ros.log.warn('pc published');
			ros.log.warn("genpc DONE");

			sens.cset(Object.assign({'TriggerMode':'Off'},param_V));
			res.message='scan compelete:'+imgs[0].length;
			res.success=true;
ros.log.warn('capture completed');
			viewOut(vue_N,vue_L,capt_L,vue_R,capt_R);
			resolve(true);
ros.log.warn("setTimeout2 function end");
			}, 200); // TODO 200固定よりもFPSから計算すべき? 「この値-p2のsetTimeout値」が 1000/FPS 以上になるように?

		});
	});
	const svc_parse=rosNode.advertiseService(NS+'/parse',rovi_srvs.dialog,(req,res)=>{
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
			ros.log.warn("cset start");
			sens.cset(obj);
			for(let key in obj){
				ros.log.info('setParam:'+NSlive+'/camera/'+key+'='+obj[key]);
				rosNode.setParam(NSlive+'/camera/'+key,obj[key]);
			}
			ros.log.warn("cset end");
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
ros.log.warn('in view N='+vue_N);
if (capt_L === undefined) {
  ros.log.warn('L undefined!!');
}
if (capt_R === undefined) {
  ros.log.warn('R undefined!!');
}
				viewOut(vue_N,vue_L,capt_L,vue_R,capt_R);
				resolve(true);
			});
		}
	});
});
