#!/usr/bin/env node

const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const geometry_msgs = ros.require('geometry_msgs').msg;
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const rovi_srvs = ros.require('rovi').srv;
const math = require('mathjs');
const EventEmitter = require('events').EventEmitter;
const Notifier = require('../script/notifier.js');

let rosNode;

class ShapeDetector{
  constructor(ns,src){
    this.params={};
    this.ev=new EventEmitter();
    const who=this;
    setImmediate(async function(){
      who.detector=rosNode.serviceClient(ns+'/do', rovi_srvs.Detect2D, { persist: true });
      if (!await rosNode.waitForService(who.detector.getService(), 2000)){
        ros.log.error('do service not available');
        return;
      }
      who.parse=rosNode.serviceClient(ns+'/parse', rovi_srvs.Dialog, { persist: true });
      if (!await rosNode.waitForService(who.parse.getService(), 2000)){
        ros.log.error('parse service not available');
        return;
      }
      const sub=rosNode.subscribe(src, sensor_msgs.Image, async (src)=>{
        who.ev.emit('img',src);
      });
    });
  }
  async set(key,val){
    let req=new rovi_srvs.Dialog.Request();
    req.hello=key+'='+JSON.stringify(val)
console.log(req.hello);
    let res=await this.parse.call(req);
  }
  solve(){
    const who=this;
    return new Promise(function(resolve){
      who.ev.once('img',async function(img){
        let req = new rovi_srvs.Detect2D.Request();
        req.img=img;
        let res = await who.detector.call(req);
        let point=res.scene[0];
        resolve([point.x,point.y]);
      });
    });
  }
}

setImmediate(async function(){
  rosNode = await ros.initNode('detector_node');
  const circle=[
    new ShapeDetector('/left/circle_detector','/rovi/left/image_rect'),
    new ShapeDetector('/right/circle_detector','/rovi/right/image_rect')
  ];
  const detectorParam=new Notifier(rosNode,'/detector');
  detectorParam.on('change',function(key,val){
    circle.forEach(function(obj){ obj.set(key,val);});
  });
  const Q=math.reshape(await rosNode.getParam('/rovi/genpc/Q'),[4,4]);
  console.log(Q);
  const pleft=rosNode.advertise('/detector/uvL', geometry_msgs.Point);
  const pright=rosNode.advertise('/detector/uvR', geometry_msgs.Point);
  const p3d = rosNode.advertise('/detector/position', geometry_msgs.Point);
  setTimeout(async function(){
    detectorParam.start();
    while(true){
      let uvs=await Promise.all([circle[0].solve(),circle[1].solve()]);
      console.log('UV='+uvs);
      let dis=[uvs[0][0],uvs[0][1],uvs[1][0]-uvs[0][0],1];
      console.log('DIS='+dis);
      let W=math.multiply(Q,math.transpose(dis));
      console.log('3D='+W);
      let pl=new geometry_msgs.Point();
      pl.x=uvs[0][0];
      pl.y=uvs[0][1];
      pleft.publish(pl);
      let pr=new geometry_msgs.Point();
      pr.x=uvs[1][0];
      pr.y=uvs[1][1];
      pright.publish(pr);
      let pnt=new geometry_msgs.Point();
      pnt.x=W[0]/W[3];
      pnt.y=W[1]/W[3];
      pnt.z=W[2]/W[3];
      p3d.publish(pnt);
    }
  },1000);
});
