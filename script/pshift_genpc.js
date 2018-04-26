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

const imgdbg = false;

ros.Time.diff=function(t0){
	let t1=ros.Time.now();
	t1.secs-=t0.secs;
	t1.nsecs-=t0.nsecs;
	return ros.Time.toSeconds(t1);
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
	const rosNode=await ros.initNode(NSthis);
	const pub_tat=rosNode.advertise(NSthis+'/tat', std_msgs.Float32);
	const pub_stat=rosNode.advertise(NSthis+'/stat', std_msgs.Bool);
	let vue_N=0;

	const raw_L=rosNode.advertise(NScamL+'/image_raw', sensor_msgs.Image);
	const rect_L=rosNode.advertise(NScamL+'/image_rect', sensor_msgs.Image);
	const vue_L=rosNode.advertise(NScamL+'/view', sensor_msgs.Image);
	const info_L=rosNode.advertise(NScamL+'/camera_info', sensor_msgs.CameraInfo);
	const remap_L=rosNode.serviceClient(NScamL+'/remap',rovi_srvs.ImageFilter,{persist:true});
	if(! await rosNode.waitForService(remap_L.getService(),2000)){
		ros.log.error('remap_L service not available');
		return;
	}
	const raw_R=rosNode.advertise(NScamR+'/image_raw', sensor_msgs.Image);
	const rect_R=rosNode.advertise(NScamR+'/image_rect', sensor_msgs.Image);
	const vue_R=rosNode.advertise(NScamR+'/view', sensor_msgs.Image);
	const info_R=rosNode.advertise(NScamR+'/camera_info', sensor_msgs.CameraInfo);
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
	let param_L,param_R;
	try{
		param_L=await rosNode.getParam(NScamL+'/camera');
		param_R=await rosNode.getParam(NScamR+'/camera');
	}
	catch(err){
		ros.log.warn('No camera params for L/R');
	}
	let param_P=await rosNode.getParam(NSthis+'/projector');
	let param_C=await rosNode.getParam(NSthis+'/camera');//-------camera param for phase shift mode
	let param_V=await rosNode.getParam(NSlive+'/camera');//-------camera param for streaming mode
	let info_l=Object.assign(new sensor_msgs.CameraInfo(),await rosNode.getParam(NScamL+'/remap'));
	let info_r=Object.assign(new sensor_msgs.CameraInfo(),await rosNode.getParam(NScamR+'/remap'));

	let sensEv;
	switch(sensName){
	case 'ycam1s':
		sensEv=sens.open(param_L.ID,param_R.ID,param_P.Url,param_P.Port,param_V);
		break;
	case 'ycam3':
		sensEv=sens.open(rosNode,NSrovi);
		break;
	}
	sensEv.on('stat',function(s){
		let f=new std_msgs.Bool();
		f.data=s;
		pub_stat.publish(f);
	});
	sensEv.on('wake',async function(s){
		param_V=await rosNode.getParam(NSlive+'/camera');
		sens.cset(param_V);
	});
	const sensHook=new EventEmitter();
	sensEv.on('left',async function(img){//--------a left eye image comes up
		if (imgdbg) {
			ros.log.warn("from ycam1s left seq=" + img.header.seq);
		}
		raw_L.publish(img);
		let req=new rovi_srvs.ImageFilter.Request();
		req.img=img;
		let res=await remap_L.call(req);
		// for raw img genpc, replace res.img with req.img
		if(sensHook.listenerCount('left')>0) sensHook.emit('left',res.img);
		else rect_L.publish(res.img);
		info_l.header=req.img.header;
		info_L.publish(info_l);
	});
	sensEv.on('right',async function(img){//<--------a right eye image comes up
		if (imgdbg) {
			ros.log.warn("from ycam1s right seq=" + img.header.seq);
		}
		raw_R.publish(img);
		let req=new rovi_srvs.ImageFilter.Request();
		req.img=img;
		let res=await remap_R.call(req);
		// for raw img genpc, replace res.img with req.img
		if(sensHook.listenerCount('right')>0) sensHook.emit('right',res.img);
		else rect_R.publish(res.img);
		info_r.header=req.img.header;
		info_R.publish(info_r);
	});

//---------Definition of services
	let capt_L;//<--------captured images of the left camera
	let capt_R;//<--------same as right
	const svc_do=rosNode.advertiseService(NSthis,std_srvs.Trigger,(req,res)=>{//<--------generate PCL
ros.log.warn('pshift_genpc called!');
		if(!sens.normal){
			ros.log.warn(res.message='YCAM not ready');
			res.success=false;
			return true;
		}
		return new Promise(async (resolve)=>{
			param_P=await rosNode.getParam(NSthis+'/projector');
			let wdt=setTimeout(function(){//<--------watch dog
				resolve(false);
				sensHook.removeAllListeners();
				sens.cset(Object.assign({'TriggerMode':'Off'},param_V));
ros.log.warn('in setTimeout');
			},param_P.Interval*20);
ros.log.warn('before cset TriggerMode:On');
			sens.cset({'TriggerMode':'On'});
ros.log.warn('after  cset TriggerMode:On');
			param_C=await rosNode.getParam(NSthis+'/camera');
			sens.cset(param_C);
			param_V=await rosNode.getParam(NSlive+'/camera');
			for(let key in param_V) if(!param_C.hasOwnProperty(key)) delete param_V[key];

ros.log.warn('now await setTimeout1');
await setTimeout(async function() {
ros.log.warn("setTimeout1 function start");
			sens.pset('x'+param_P.ExposureTime);
			sens.pset((sensName.startsWith('ycam3')? 'o':'p')+param_P.Interval);
			let val=param_P.Intencity<256? param_P.Intencity:255;
			val=val.toString(16);
			sens.pset('i'+val+val+val);
ros.log.warn('before pset p2');
			sens.pset(sensName.startsWith('ycam3')? 'o2':'p2');//<--------projector sequence start
ros.log.warn('after  pset p2');
ros.log.warn("setTimeout1 function end");
			}, 125); // これはcsetが実際に反映されるのを待つ時間も兼ねる(ライブの残りカスを捨てるのも)

ros.log.warn('now await setTimeout2');
await setTimeout(async function() {
ros.log.warn("setTimeout2 function start");
			let imgs=await Promise.all([
				new Promise((resolve)=>{
					let capt=[];
					ros.log.warn("before sensHook.on('left')");
					sensHook.on('left',function(img){
ros.log.warn('capturing img_L:'+capt.length+" seq="+img.header.seq);
						if (imgdbg) {
							ros.log.warn("capt ycam1s left seq=" + img.header.seq + " ... " + capt.length);
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
					ros.log.warn("after  sensHook.on('left'");
				}),
				new Promise((resolve)=>{
					let capt=[];
//					resolve(capt);
					ros.log.warn("before sensHook.on('right')");
					sensHook.on('right',function(img){
ros.log.warn('capturing img_R:'+capt.length+" seq="+img.header.seq);
						if (imgdbg) {
							ros.log.warn("capt ycam1s right seq=" + img.header.seq + " ... " + capt.length);
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
					ros.log.warn("after  sensHook.on('right'");
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
			}, 250); // TODO 250固定よりもFPSから計算すべき? 「この値-p2のsetTimeout値」が 1000/FPS 以上になるように?

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
