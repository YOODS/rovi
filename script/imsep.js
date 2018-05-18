#!/usr/bin/env node

const fs=require('fs');
const ros=require('rosnodejs');
const sensor_msgs=ros.require('sensor_msgs').msg;

function topgm(img,fn){
	let cap=new Uint8Array(img.data);
	return new Promise(function(resolve){
		fs.writeFileSync(fn+'.pgm','P5\n'+img.width.toString()+' '+img.height.toString()+'\n'+'255\n');
		fs.appendFileSync(fn+'.pgm',cap);
		resolve(true);
	});
}

setImmediate(async function(){
	if(process.argv.length<3){
		ros.log.error('Missing target topic name to subscribe');
		return;
	}
	const rosNode=await ros.initNode('/imsep');
	const image_l=new sensor_msgs.Image();
	const image_r=new sensor_msgs.Image();
	image_l.data=new Uint8Array();
	image_r.data=new Uint8Array();
	const rect_l=rosNode.advertise('/left/image_rect', sensor_msgs.Image);
	const rect_r=rosNode.advertise('/right/image_rect', sensor_msgs.Image);
	let sub=rosNode.subscribe(process.argv[2],sensor_msgs.Image,async (src)=>{
		const h=image_l.height=image_r.height=src.height;
		const w=image_l.step=image_r.step=src.step/2;
		image_l.header=image_r.header=src.header;
		image_l.width=image_r.width=src.width/2;
		image_l.encoding=image_r.encoding=src.encoding;
		let sz=image_l.step*image_l.height;
		if(sz>image_l.data.byteLength) image_l.data=new Uint8Array(sz);
		if(sz>image_r.data.byteLength) image_r.data=new Uint8Array(sz);
		for(let i=0,s=0,t=0;i<h;i++,t+=w){
			image_l.data.subarray(t).set(src.data.subarray(s,s+w));
			s+=w;
			image_r.data.subarray(t).set(src.data.subarray(s,s+w));
			s+=w;
		}
		rect_l.publish(image_l);
		rect_r.publish(image_r);
	});
});
