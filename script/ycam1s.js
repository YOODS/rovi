const Net=require('net');
const EventEmitter=require('events').EventEmitter;
const Runner=require('./runner.js');
const Notifier=new EventEmitter();

const ros=require('rosnodejs');
const sensor_msgs=ros.require('sensor_msgs').msg;
const std_msgs=ros.require('std_msgs').msg;
const std_srvs=ros.require('std_srvs').srv;
const rovi_srvs=ros.require('rovi').srv;

const shm=require('shm-typed-array');
let shmem;

let run_c;  //should be rosrun.js camera runner
let run_p;  //should be openYPJ:Socket

let imgLength;
function imgCreate(param){
	let img=new sensor_msgs.Image();
	img.header.seq=0;
	img.header.stamp=0;
	img.header.frame_id='ycam1s';
	img.width=1280;
	img.height=1024;
	img.step=1280;
	img.encoding='mono8';
	for(let key in img){
		if(param.hasOwnProperty(key)){
			img[key]=param[key];
		}
	}
	imgLength=img.height*img.step;
	return img;
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

let image_l;
let image_r;

function msleep(t){
	return new Promise(function(resolve){
		setTimeout(function(){
			resolve(true);
		},t);
	});
}
var ycam={
	cset:function(obj){
		console.log("yam1s cset called");
		for(let key in obj){
			console.log("ycam1s cset key="+key+",val="+obj[key]);
			run_c.cin(key+' '+obj[key]);
		}
		return true;
	},
	pset:function(str){
		run_p.write(str+'\n');
		run_p.setNoDelay(true);
	},
	normal:false,
	stat:function(){
		return {'camera':run_c.running,'projector':!run_p.destroyed};
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
	open:function(idl,idr,url,port,param_V){
//		run_c=Runner.run('grabber-sentech '+idl+' '+idr);
//		run_c=Runner.run('../basler_example/grabber');
		run_c=Runner.run(process.env.ROVI_PATH + "/sentech_grabber/grabber '" + idl + "' '" + idr + "'");
		this.scan();
		run_c.on('start',function(data){
			console.log("get start");
			ycam.cset(param_V);
		});
		run_c.on('cout',function(data){
			let lines = data.split(/\n/);
			for (let i = 0; i < lines.length; i++) {
				if (!lines[i]) {
					continue;
				}
				let attr;
				try{
					attr=JSON.parse(lines[i]);
//					ros.log.warn('ycam1s done parse [' + lines[i] + ']');
				}
				catch(err){
					ros.log.error('ycam1s failed parse [' + lines[i] + ']');
					ros.log.error('ycam1s parse err[' + err +']');
					return;
				}
				if(attr.hasOwnProperty('capt')){
					let offset=attr.capt;
					if(offset==0){
						image_l.header.seq++;
						image_l.header.stamp=ros.Time.now();
						image_l.data=shmem.slice(0,imgLength);
						Notifier.emit('cam_l', copyImg(image_l));
					}
					else{
						image_r.header.seq++;
						image_r.header.stamp=ros.Time.now();
						image_r.data=shmem.slice(offset,offset+imgLength);
						Notifier.emit('cam_r', copyImg(image_r));
					}
				}
				else if(attr.hasOwnProperty('shm')){
					shmem=shm.get(attr.shm,'Uint8Array');
					delete attr.shm;
					image_l=imgCreate(attr);
					image_r=imgCreate(attr);
console.log('shm size:'+shmem.byteLength);
				}
			}
		});
		run_p=openYPJ(port,url);
		run_p.on('data',function(data){
			if(data=='Dlp.X>'){
			}
		});
		return Notifier;
	}
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
	sock.connect(port,url);
	return sock;
}

module.exports=ycam;
