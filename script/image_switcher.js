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
    this.pstat=0;   //0:live,1:settling,2:pshift
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
  async emit(img){
    switch(this.pstat){
    case 0:
      this.raw.publish(img);
      let req=new rovi_srvs.ImageFilter.Request();
      req.img=img;
      let res=await this.remap.call(req);
      this.rect.publish(res.img);
      break;
    case 2:
      this.hook.emit('store',img);
      break;
    }
  }
  store(count) {
    if(this.pstat!=0) return Promise.reject(new Error('genpc busy'));
    const who = this;
    this.capt = [];
    this.pstat=2;

    return new Promise(function(resolve){
      who.hook.on('store', async function(img){
        let icnt=who.capt.length;
        who.capt.push(img);
        icnt++;
        if(icnt==count){
          who.pstat=3;
          for(let i=0;i<count;i++){
            let req=new rovi_srvs.ImageFilter.Request();
            req.img=who.capt[i];
            let res=await who.remap.call(req);
            who.capt[i]=res.img;
          }
          who.rect.publish(who.capt[1]);
          setTimeout(function(){ if(who.pstat==3) who.thru();},1000);
          resolve(who.capt);
        }
      });
    });
  }
  thru() {
    this.hook.removeAllListeners();
    this.pstat=0;
  }
  get ID() {return this.param.ID;}
  view(n) {
    if (n < this.capt.length) {
      this.vue.publish(this.capt[n]);
    }
  }
}

module.exports=ImageSwitcher;

