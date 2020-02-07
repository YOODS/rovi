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
    this.rect0 = node.advertise(ns + '/image_rect0', sensor_msgs.Image);
    this.rect1 = node.advertise(ns + '/image_rect1', sensor_msgs.Image);
    this.diff = node.advertise(ns + '/diff_rect', sensor_msgs.Image);     // WPC
    this.vue = node.advertise(ns + '/view', sensor_msgs.Image);
    this.info = node.advertise(ns + '/camera_info', sensor_msgs.CameraInfo);
    this.remap = node.serviceClient(ns + '/remap', rovi_srvs.ImageFilter, { persist: true });
    this.pstat=0;   //0:live,1:settling,2:pshift
    setImmediate(async function() {
      if (!await node.waitForService(who.remap.getService(), 2000)) {
        ros.log.error('remap service not available');
        return;
      }
    });
    this.hook = new EventEmitter();
    this.capt = [];
  }
  async emit(img,ts,lit){
    switch(this.pstat){
    case 0:
      this.raw.publish(img);
      let req=new rovi_srvs.ImageFilter.Request();
      req.img=img;
      let res=await this.remap.call(req);
      if(lit) this.rect.publish(res.img);
      else this.rect0.publish(res.img);
      //this.diff.publish(res.img);                                              // WPC
      break;
    case 2:
      this.hook.emit('store',img,ts);
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
          let tmp =who.capt[0];
          who.pimgF=img;
          let buf=new sensor_msgs.Image();
          buf.header=img.header;
          buf.height=img.height;
          buf.width=img.width;
          buf.encoding=img.encoding;
          buf.is_bigendian=img.is_bigendian;
          buf.step=img.step;
          buf.data=Array(img.data.length);
          let d=who.capt[1].data-who.capt[0].data;
          for (var i=0; i<who.capt[1].data.length; i++) {
            let d=who.capt[1].data[i]-who.capt[0].data[i];
            if (d<1) { buf.data[i]=0 }
            else if (d>255) { buf.data[i]=255 }
            else { buf.data[i]=d };
          }
          who.rect0.publish(who.capt[0]);
          who.rect1.publish(who.capt[1]);
          who.rect.publish(who.capt[1]);
          who.diff.publish(buf);
          resolve(who.capt);
        }
      });
    });
  }
  thru() {
    this.hook.removeAllListeners();
    this.pstat=0;
  }
  view(n) {
    if (n < this.capt.length) {
      this.vue.publish(this.capt[n]);
    }
  }
}

module.exports=ImageSwitcher;

