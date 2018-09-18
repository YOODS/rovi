const Net = require('net');
const EventEmitter = require('events').EventEmitter;
const RosrunL = require('./rosrun.js');
const RosrunR = require('./rosrun.js');
const Notifier = new EventEmitter;

const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const dyn_msgs = ros.require('dynamic_reconfigure').msg;
const dyn_srvs = ros.require('dynamic_reconfigure').srv;
const rovi_srvs = ros.require('rovi').srv;

let run_l;  // should be rosrun.js camnode left
let run_r;  // should be rosrun.js camnode right
let run_p;  // should be openYPJ:Socket
let rosNode;
let camera_l = '/camera/';
let camera_r = '/camera/';

diffTime = function(t1, t0) {
  let dt = ros.Time.now();
  dt.secs = t1.secs - t0.secs;
  return ros.Time.toSeconds(dt);
}

var ycam = {
  isliveon: async function() {
    // TODO
    let ison = true;
    return ison;
  },
  cset: async function(obj) {
    let request = new dyn_srvs.Reconfigure.Request();
    for (let key in obj) {
      let val = obj[key];
      if (typeof(val) == 'string') {
        let param = new dyn_msgs.StrParameter();
        param.name = key;
        param.value = val;
        request.config.strs.push(param);
console.log('ycam.cset as string:' + key + '=' + val);
      }
      else {
        let param = new dyn_msgs.DoubleParameter();
        param.name = key;
        param.value = val;
        request.config.doubles.push(param);
console.log('ycam.cset as double:' + key + '=' + val);
      }
    }
    let res_l = await run_l.dynparam_set.call(request);
    let res_r = await run_r.dynparam_set.call(request);
    return 'OK';
  },
  pset: function(obj) {
    for (let key in obj) {
      switch(key){
      case 'ExposureTime':
        run_p.write('x' + obj[key]+'\n');
        break;
      case 'Interval':
        run_p.write('p' + obj[key]+'\n');
        break;
      case 'Intencity':
        let ix = obj[key] < 256 ? obj[key] : 255;
        ix=ix.toString(16);
        run_p.write('i'+ix+ix+ix+'\n');
        break;
      case 'Go':
        let gx= obj[key]<2? obj[key]:2;
        run_p.write('p'+gx+'\n');
        break;
      }
      run_p.setNoDelay(true);
    }
    return 'OK';
  },
  normal: false,
  stat: function() {
    let fl=false,fr=false,fp=false;
    try{
      fl=run_l.running;
      fr=run_r.running;
      fp=!run_p.destroyed
    }
    catch(e){
      ros.log.error(e);
    }
    return { 'left': fl, 'right': fr, 'projector': fp };
  },
  scan: function() {
    let s;
    try {
      s = this.stat();
    }
    catch(err) {
      Notifier.emit('stat', this.normal = false);
      setTimeout(function() {ycam.stat();}, 1000);
      return;
    }
    let f = true;
    for (let key in s) f = f && s[key];
    if (f == undefined) f = false;
    Notifier.emit('stat', this.normal = f);
    setTimeout(function() {ycam.scan();}, 1000);
  },
  open: function(nh, nsl, idl, nsr, idr, url, port) {
    rosNode = nh;
    camera_l = nsl + '/camera/';
    camera_r = nsr + '/camera/';
    run_l = RosrunL.run('camera_aravis camnode ' + idl, nsl);
    let who=this;
    run_l.on('start', function() {
      openCamera(run_l, camera_l, 'left');
      run_r = RosrunR.run('camera_aravis camnode ' + idr, nsr);
      run_r.on('start', function() {
        openCamera(run_r, camera_r, 'right');
        run_p = openYPJ(port, url);
        run_p.on('connect',function(){
          console.log('ycam1h-emit-wake');
          Notifier.emit('wake');
          who.scan();
        });
      });
    });
    return Notifier;
  }
}

async function openCamera(rosrun, ns, evname) {
  let sub = rosNode.subscribe(ns + 'image_raw', sensor_msgs.Image, (src) => {
    if (diffTime(src.header.stamp, rosrun.timestamp) >= 0.0) {
      Notifier.emit(evname, src);
      rosrun.timestamp = src.header.stamp;
    }
  });
  var cset = rosNode.serviceClient(ns + 'set_parameters', dyn_srvs.Reconfigure, { persist: true });
  if (!await rosNode.waitForService(cset.getService(), 2000)) {
    ros.log.error('Service not available');
    return;
  }
  rosrun.dynparam_set = cset;
  rosrun.timestamp = ros.Time.now();
}

function openYPJ(port, url, sock) {
  if (arguments.length < 3) sock = new Net.Socket();
  sock.on('connect', function() {
    ros.log.info('***YPJ connected***');
  });
  sock.on('error', function() {
    ros.log.error('YPJ error');
  });
  sock.on('close', function() {
    ros.log.info('YPJ closed');
    setTimeout(function() {
      sock.connect(port, url);
    }, 3000);
  });
  sock.on('data', function(data) {
    Notifier.emit('pout', data);
  });
  sock.on('drain', function(data) {
    console.log('YPJ drain');
  });
  sock.connect(port, url);
  return sock;
}

module.exports = ycam;
