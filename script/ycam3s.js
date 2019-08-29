const Net = require('net');
const EventEmitter = require('events').EventEmitter;
const Rosrun = require('./rosrun.js');
const Notifier = new EventEmitter;
const terminate=require('terminate');

const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const dyn_msgs = ros.require('dynamic_reconfigure').msg;
const dyn_srvs = ros.require('dynamic_reconfigure').srv;
const gev_srvs = ros.require('camera_aravis').srv;
const rovi_srvs = ros.require('rovi').srv;

const execSync = require('child_process').execSync;

const shm=require('shm-typed-array');

let run_c;  // should be rosrun.js camnode
let rosNode;

function sleep(msec){return new Promise(function(resolve){setTimeout(function(){resolve(true);},msec);});}

const reg_table = {
  'AcquisitionStart': 0x00100018,
  'AcquisitionStop': 0x0010001c,
  'AcquisitionFrameRate': 0x00101000,
  'ExposureTime': 0x00101004,
  'GainAnalog': 0x00101008,
  'Gain': 0x0010100C,
  'TriggerMode': 0x00101010,
  'SerialPort': 0x00300004,
  'SoftwareTriggerRate': 0
}

const val_table = {
  'Off': 0,
  'On': 1
}

var ycam = {
  cset: async function(obj) {
    if(!ycam.cstat) return 'ycam3s not ready';
    let ret = 'OK';
    let greq = new gev_srvs.GevRegs.Request();
    let dreq = new dyn_srvs.Reconfigure.Request();
    for (let key in obj) {
      let val = obj[key];
      if (reg_table.hasOwnProperty(key)) {
        greq.address = reg_table[key];
        if(greq.address!=0){
          if(typeof(val)=='string'){
            if(val_table.hasOwnProperty(val)) greq.data=val_table[val];
            else greq.data=Number(val);
          }
          else greq.data=val;
          try {
            await run_c.reg_write.call(greq);
          }
          catch(err) {
            ros.log.error('YCAM3 cset write '+err+' at '+key);
            ret = 'YCAM write reg failed';
          }
        }
      }
      else if (typeof(val) == 'string') {
        let param = new dyn_msgs.StrParameter();
        param.name = key;
        param.value = val;
        dreq.config.strs.push(param);
      }
      else {
        let param = new dyn_msgs.DoubleParameter();
        param.name = key;
        param.value = val;
        dreq.config.doubles.push(param);
      }
    }
    if (dreq.config.strs.length > 0 || dreq.config.doubles.length > 0) {
      await run_c.dynparam_set.call(dreq);
    }
    return ret;
  },
  pstat:false,
  cstat:false,
  pregbuf: '',
  pregwrt: async function() {
    let success = true;
    if (ycam.pregbuf.length > 0) {
      let greq = new gev_srvs.GevRegs.Request();
      greq.address = reg_table['SerialPort'];
      let lsb = ycam.pregbuf.charCodeAt(0);
      greq.data = (~lsb << 16) | lsb;
      try {
        await run_c.reg_write.call(greq);
        ycam.pstat=true;
        ycam.pregbuf=ycam.pregbuf.slice(1);
        setImmediate(function(){ ycam.pregwrt();});
      }
      catch(err) {
        ros.log.error('YCAM3 pregwrt "'+lsb+'" '+err);
        ycam.pstat=false;
        setTimeout(function(){
          if(!ycam.cstat) return;
          ros.log.error('YCAM3 pregwrt retry');
          ycam.pregwrt();
        },100);
      }
    }
  },
  pset: async function(obj) {
    if(!ycam.cstat) return 'ycam3s not ready';
    let ret = 'OK';
    let str = '';
    for (let key in obj) {
      switch (key) {
      case 'ExposureTime':
          str += 'x' + obj[key] + '\n';
          break;
        case 'Interval':
          str += 'o' + obj[key] + '\n';
          break;
      case 'Intencity':
        let ix = obj[key] < 256 ? obj[key] : 255;
        ix=('00'+ix.toString(16).toUpperCase()).substr(-2);
        str += 'i' + ix + ix + ix + '\n';
        break;
      case 'Go':
        str += 'o' + obj[key] + '\n';
        break;
      case 'Inv':
        str += 'b' + obj[key] + '\n';
        break;
      case 'Mode':
        str += 'z' + obj[key] + '\n';
        break;
      case 'Reset':
      case 'Init':
        ycam.pregbuf='';
        str = '#\n';
        break;
      }
    }
    const bs=ycam.pregbuf.length;
    ycam.pregbuf += str;
    if(bs>0 || !ycam.pstat) return ret;
    ycam.pregwrt();
    return ret;
  },
  normal: false,
  stat: function() {
    return {'camera':ycam.cstat};
  },
  scan: function() {
    let s;
    try {
      s = ycam.stat();
    }
    catch(err) {
      Notifier.emit('stat', ycam.normal = false);
      setTimeout(function() {ycam.scan();}, 1000);
      return;
    }
    let f = true;
    for (let key in s) f = f && s[key];
    if (f == undefined) f = false;
    Notifier.emit('stat', this.normal = f);
    setTimeout(function() {ycam.scan();}, 1000);
  },
  open: async function(nh, ns) {
    const who=this;
    ros.log.warn('YCAM3 Opening...');
    rosNode = nh;
    run_c = Rosrun.run('camera_aravis camnode', ns);  //Rosrun:childprocess
    run_c.on('start', async function() {
      if (!await openCamera(run_c, ns)) {
        ros.log.error('Failure in openCamera');
        process.exit(101);
      }
      who.pset({Init:1});
      Notifier.emit('wake');
      setTimeout(function(){
        ycam.cstat=ycam.pstat=true;
        ycam.pregbuf='';
      },1000);
    });
    run_c.on('stop', async function() {
      Notifier.emit('shutdown','camera stopped');
      ros.log.warn('YCAM3 Stopped');
      ycam.cstat=ycam.pstat=false;
      ycam.pregbuf='';
      rosNode.unsubscribe(ns + '/camera/image_raw');
    });
    this.scan();
    Notifier.device=ycam;
    return Notifier;
  },
  kill: function(){
console.log("kkill"+run_c.running);
    if(run_c.running!=null){// process.kill(-run_c.running.pid);
      terminate(run_c.running.pid,function(err){
        console.log('terminate '+err);
      });
    }
  }
}

async function openCamera(rosrun, ns) {
  let shmptr;
  let shmkey=0;
  let sub = rosNode.subscribe(ns + '/camera/image_raw', sensor_msgs.Image, (src) => {
    const buffer=Buffer.from(src.data);
    const nkey=buffer.readUInt32LE(0);
//    ros.log.info("shmkey="+Number(nkey).toString(16));
    if(nkey!=shmkey){
      try{
        if(shmkey!=0) shm.detach(shmkey);
        shmptr=shm.get(shmkey=nkey,'Uint8Array');
      }
      catch(err){
        console.log('get:'+err);
      }
    }
    src.data = new Uint8Array(src.height*src.step);
    src.data.set(shmptr);
    let image_l = new sensor_msgs.Image();
    let image_r = new sensor_msgs.Image();
    const h = image_l.height = image_r.height = src.height;
    const w = image_l.step = image_r.step = src.step / 2;
    image_l.header = image_r.header = src.header;
    image_l.width = image_r.width = src.width / 2;
    image_l.encoding = image_r.encoding = src.encoding;
    let sz = image_l.step * image_l.height;
    image_l.data = new Uint8Array(sz);
    image_r.data = new Uint8Array(sz);
    for (let i = 0, s = 0, t = 0; i < h; i++, t += w) {
      image_l.data.subarray(t).set(src.data.subarray(s, s + w));
      s += w;
      image_r.data.subarray(t).set(src.data.subarray(s, s + w));
      s += w;
    }
    let ts=ros.Time.now();
    Notifier.emit('left', image_l,ts);
    Notifier.emit('right', image_r,ts);
  });
  return new Promise(async function(resolve) {
    let regw = rosNode.serviceClient(ns + '/camera/regw', gev_srvs.GevRegs, { persist: true });
    if (await rosNode.waitForService(regw.getService(), 2000)) {
      rosrun.reg_write = regw;
    }
    else {
      ros.log.error(ns + '/"regw" not available');
      resolve(false);
    }
    let regr = rosNode.serviceClient(ns + '/camera/regr', gev_srvs.GevRegs, { persist: true });
    if (await rosNode.waitForService(regr.getService(), 2000)) {
      rosrun.reg_read = regr;
    }
    else {
      ros.log.error(ns + '/"regr" not available');
      resolve(false);
    }
    let cset = rosNode.serviceClient(ns + '/camera/set_parameters', dyn_srvs.Reconfigure, { persist: true });
    if (await rosNode.waitForService(cset.getService(), 2000)) {
      rosrun.dynparam_set = cset;
    }
    else {
      ros.log.error(ns + '/"set_parameters" not available');
      resolve(false);
    }
    resolve(true);
  });
}

module.exports = ycam;

