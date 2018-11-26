const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const rovi_srvs = ros.require('rovi').srv;
const EventEmitter = require('events').EventEmitter;

class ImageSwitcher {
  constructor(node, ns) {
    const who = this;
    this.node = node;
    this.ns = ns;
    this.raw = node.advertise(ns + '/image_raw', sensor_msgs.Image);
    this.rect = node.advertise(ns + '/image_rect', sensor_msgs.Image);
    this.vue = node.advertise(ns + '/view', sensor_msgs.Image);
    this.info = node.advertise(ns + '/camera_info', sensor_msgs.CameraInfo);
    this.remap = node.serviceClient(ns + '/remap', rovi_srvs.ImageFilter, { persist: true });
    this.imgqueue=[];
    this.pstat=0;   //0:live,1:settling,2:pshift
    this.pimg;
    setImmediate(async function() {
      if (!await node.waitForService(who.remap.getService(), 2000)) {
        ros.log.error('remap service not available');
        return;
      }
      try {
        who.param = await node.getParam(ns + '/camera');
      }
      catch(err) {
        ros.log.warn('getParam ' + err);
      }
    });
    this.hook = new EventEmitter();
    this.capt = [];
  }
  async imgproc(){
    let img=this.imgqueue[0];
    let req = new rovi_srvs.ImageFilter.Request();
    req.img = img;
    let res;
    try {
      switch(this.pstat){
      case 0:
        this.raw.publish(img);
        res=await this.remap.call(req);
        this.rect.publish(res.img);
        break;
      case 2:
        res=await this.remap.call(req);
        this.hook.emit('store', res.img);
        break;
      }
      this.imgqueue.shift();
    }
    catch(err) {
      ros.log.error('ImageSwitcher::emit "remap failed:"'+err);
      this.imgqueue=[];
    }
    if(this.imgqueue.length>0){
      let who=this;
      setImmediate(function(){
        who.imgproc();
      });
    }
  }
  async emit(img) {
    this.imgqueue.push(img);
    if(this.imgqueue.length==1){
      let who=this;
      setImmediate(function(){
        who.imgproc();
      });
    }
  }
  store(count,delay) {
    if(this.pstat!=0) return Promise.reject(new Error('genpc busy'));
    const who = this;
    this.capt = [];
    this.pstat=1;
    setTimeout(function(){ who.pstat=2;},delay);
    return new Promise(function(resolve) {
      who.hook.on('store', function(img) {
        let icnt=who.capt.length;
        console.log(who.ns+':'+icnt);
        who.capt.push(img);
        icnt++;
        if(icnt==2) who.pimg=img;
        else if(icnt==count){
          who.pstat=3;
          who.rect.publish(who.pimg);
          resolve(who.capt);
        }
//        else if(icnt>2) who.rect.publish(who.pimg);
      });
    });
  }
  cancel() {
    let who=this;
    let ts=arguments.length>0? arguments[0]:1;
    this.hook.removeAllListeners();
    this.pstat=3;
    setTimeout(function(){
      who.pstat=0;
    },ts);
  }
  get ID() {return this.param.ID;}
  view(n) {
    if (n < this.capt.length) {
      this.vue.publish(this.capt[n]);
    }
  }
}

module.exports=ImageSwitcher;

