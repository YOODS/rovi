#!/usr/bin/env node

const NSycamctrl = '/rovi/ycam_ctrl';
const NSpsgenpc = '/rovi/pshift_genpc';
const NSgenpc = '/rovi/genpc';
const NScamL = '/rovi/left';
const NScamR = '/rovi/right';
const NSlive = '/rovi/live';
const NSrovi = '/rovi';
const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const sensName = process.argv.length < 3 ? 'ycam1s' : process.argv[2];
const sens = require('./' + sensName + '.js')
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const rovi_srvs = ros.require('rovi').srv;
const EventEmitter = require('events').EventEmitter;
const jsyaml = require('js-yaml');

let prev_sensstat = false;

ros.Time.diff = function(t0) {
  let t1 = ros.Time.now();
  t1.secs -= t0.secs;
  t1.nsecs -= t0.nsecs;
  return ros.Time.toSeconds(t1);
}

function sleep(n){
  return new Promise(function(resolve){
    setTimeout(function(){
      resolve(true);
    },n);
  });
}

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
    this.remapreload = node.serviceClient(ns + '/remap/reload', std_srvs.Trigger);
    this.imgqueue=[];
    this.pstat=0;   //0:live,1:settling,2:pshift
    this.pimg;
    setImmediate(async function() {
      if (!await node.waitForService(who.remap.getService(), 2000)) {
        ros.log.error('remap service not available');
        return;
      }
      if (!await node.waitForService(who.remapreload.getService(), 2000)) {
        ros.log.error('remap reload service not available');
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
    if (ns === NScamL) {
      this.lr = 'L';
    }
    else {
      this.lr = 'R';
    }
  }
  async imgproc(){
    let img=this.imgqueue[0];
    this.raw.publish(img);
    let req = new rovi_srvs.ImageFilter.Request();
    req.img = img;
    let res;
    try {
      switch(this.pstat){
      case 0:
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
          resolve(who.capt);
        }
        else if(icnt>2) who.rect.publish(who.pimg);
      });
    });
  }
  cancel() {
    let who=this;
    this.hook.removeAllListeners();
    this.pstat=3;
    setTimeout(function(){
      who.pstat=0;
    },1000);
  }
  get ID() {return this.param.ID;}
  view(n) {
    if (n < this.capt.length) {
      this.vue.publish(this.capt[n]);
    }
  }
  async reloadRemap() {
    let res = await this.remapreload.call(new std_srvs.Trigger.Request());
    this.caminfo = Object.assign(new sensor_msgs.CameraInfo(), await this.node.getParam(this.ns + '/remap'));
    return res;
  }
  
}

setImmediate(async function() {
  const rosNode = await ros.initNode(NSycamctrl);
  let cam_resolution;

  if (sensName === 'ycam3') {
    let camera_size = await rosNode.getParam(NSrovi + '/camera');
    if (camera_size.Height == 480 && camera_size.Width == 1280) {
      ros.log.warn('camera size = VGA');
      cam_resolution = 'vga';
    }
    else if (camera_size.Height == 1024 && camera_size.Width == 2560) {
      ros.log.warn('camera size = SXGA');
      cam_resolution = 'sxga';
    }
    else {
      ros.log.error('Invalid camera size');
      return;
    }
  }

  const image_L = new ImageSwitcher(rosNode, NScamL);
  const image_R = new ImageSwitcher(rosNode, NScamR);
  const pub_stat = rosNode.advertise(NSycamctrl + '/stat', std_msgs.Bool);
  let vue_N = 0;
  const genpc = rosNode.serviceClient(NSrovi + '/genpc', rovi_srvs.GenPC, { persist: true });
  const genpcreload = rosNode.serviceClient(NSrovi + '/genpc/reload', std_srvs.Trigger);
  if (!await rosNode.waitForService(genpc.getService(), 2000)) {
    ros.log.error('genpc service not available');
    return;
  }
  if (!await rosNode.waitForService(genpcreload.getService(), 2000)) {
    ros.log.error('genpc reload service not available');
    return;
  }
  let param_C = await rosNode.getParam(NSpsgenpc + '/camera');
  let param_V = await rosNode.getParam(NSlive + '/camera');
  let param_P = await rosNode.getParam(NSpsgenpc + '/projector');
  let paramTimer = null;
  function paramDiff(o, n) {
    let c = Object.assign({}, n);
    for (let key in c) {
      if (o.hasOwnProperty(key)) {
        if (o[key] == c[key]) delete c[key];
      }
    }
    return c;
  }
  async function paramReloadNow() {
    let ret = false;
    param_C = await rosNode.getParam(NSpsgenpc + '/camera');
    let nv = await rosNode.getParam(NSlive + '/camera');
    let np = await rosNode.getParam(NSpsgenpc + '/projector');
    try {
      let csetret = await sens.cset(paramDiff(param_V, nv));
      param_V = nv;
      let psetret = await sens.pset(paramDiff(param_P, np));
      param_P = np;
      let rrLret = await image_L.reloadRemap();
      let rrRret = await image_R.reloadRemap();
      let grret = await genpcreload.call(new std_srvs.Trigger.Request());
      if (csetret === 'OK' && psetret === 'OK' && rrLret.success && rrRret.success && grret.success) {
        ret = true;
      }
    }
    catch(err) {
      let errmsg = 'Exception in paramReloadNow:' + err;
      ros.log.error(errmsg);
      return false;
    }
    return ret;
  }
  async function paramReload() {
    param_C = await rosNode.getParam(NSpsgenpc + '/camera');
    let nv = await rosNode.getParam(NSlive + '/camera');
    let np = await rosNode.getParam(NSpsgenpc + '/projector');
    if (paramTimer != null) {
      try {
        await sens.cset(paramDiff(param_V, nv));
        param_V = nv;
        await sens.pset(paramDiff(param_P, np));
        param_P = np;
      }
      catch(err) {
        ros.log.warn('Exception in paramReload:' + err);
      }
    }
    if (paramTimer != null) paramTimer = setTimeout(paramReload, 1000);
  }
  function paramScan() {
    if (paramTimer == null) paramTimer = setTimeout(paramReload, 1000);
  }
  function paramStop() {
    if (paramTimer != null) {
      clearTimeout(paramTimer);
    }
    paramTimer = null;
  }

  let sensEv;
  switch (sensName) {
  case 'ycam1s':
    sensEv = sens.open(image_L.ID, image_R.ID, param_P.Url, param_P.Port);
    break;
  case 'ycam3':
    sensEv = sens.open(rosNode, NSrovi, cam_resolution);
    break;
  case 'ycam1h':
    sensEv = sens.open(rosNode,NScamL,image_L.ID,NScamR,image_R.ID, param_P.Url, param_P.Port);
    break;

  }
  sensEv.on('stat', function(s) {
    let f = new std_msgs.Bool();
    f.data = s;
    pub_stat.publish(f);
    if (prev_sensstat != s) {
      ros.log.warn('YCAM stat becomes ' + s);
    }
    prev_sensstat = s;
  });
  sensEv.on('wake', async function(yamlstr) {
    if (sensName === 'ycam3') {
      const yamlval = jsyaml.safeLoad(yamlstr);
      await rosNode.setParam(NScamL, yamlval.left);
      await rosNode.setParam(NScamR, yamlval.right);
      await rosNode.setParam(NSgenpc, yamlval.genpc);
    }
    param_V = {};
    param_P = {};
    paramStop();
    let prmret = await paramReloadNow();
    await sens.cset({ 'TriggerMode': 'Off' });
    paramScan();
    if (prmret) {
      ros.log.warn('NOW ALL READY');
    }
    else {
      ros.log.error('param reload ERROR');
      process.exit(99);
    }
  });
  sensEv.on('left', async function(img) {
    image_L.emit(img);
  });
  sensEv.on('right', async function(img) {
    image_R.emit(img);
  });

  paramScan();

// ---------Definition of services
  let capt_L; // <--------captured images of the left camera
  let capt_R; // <--------same as right
  const svc_do = rosNode.advertiseService(NSpsgenpc, std_srvs.Trigger, (req, res) => { // <--------generate PCL
    paramStop();
    if (!sens.normal) {
      ros.log.warn(res.message = 'YCAM not ready');
      res.success = false;
      paramScan();
      return true;
    }
    return new Promise(async (resolve) => {
      const timeoutmsec = param_P.Interval*13*2;
      let wdt = setTimeout(async function() { // <--------watch dog
        image_L.cancel();
        image_R.cancel();
        await sens.cset({ 'TriggerMode': 'Off' });
        paramScan();
        const errmsg = 'pshift_genpc timed out';
        ros.log.error(errmsg);
        res.success = false;
        res.message = errmsg;
        resolve(true);
      }, timeoutmsec);
      param_V = Object.assign(param_V, param_C);
      await sens.cset({ 'TriggerMode': 'On' });
      await sens.cset(param_C);
      const settletm=Math.floor(1000.0/param_V.AcquisitoionFrameRate);
    	setTimeout(function(){
    		sens.pset({ 'Go': 2 }); // <--------projector sequence start
	    },settletm+500);
      let imgs=await Promise.all([image_L.store(13,settletm),image_R.store(13,settletm)]);
      clearTimeout(wdt);
      capt_L = imgs[0];
      capt_R = imgs[1];
      let gpreq = new rovi_srvs.GenPC.Request();
      gpreq.imgL = capt_L;
      gpreq.imgR = capt_R;
      try {
        let gpres = await genpc.call(gpreq);
        res.message = imgs[0].length + ' images scan compelete. Generated PointCloud Count=' + gpres.pc_cnt;
        res.success = true;
      }
      catch(err) {
//      ros.log.error('genpc failed. ' + err);
        res.message = 'genpc failed';
        res.success = false;
      }
      await sens.cset({ 'TriggerMode': 'Off' });
      paramScan();
      image_L.view(vue_N);
      image_R.view(vue_N);
      image_L.cancel();
      image_R.cancel();
      resolve(true);
    });
  });

  const svc_parse = rosNode.advertiseService(NSycamctrl + '/parse', rovi_srvs.Dialog, async (req, res) => {
    let cmd = req.hello;
    let lbk = cmd.indexOf('{');
    let obj = {};
    if (lbk > 0) {
      cmd = req.hello.substring(0, lbk).trim();
      try {
        obj = JSON.parse(req.hello.substring(lbk));
      }
      catch(err) {
        // ignore
      }
    }
    let cmds = cmd.split(' ');
    if (cmds.length > 1) cmd = cmds.shift();
    switch (cmd) {
    case 'cset':
      if (!sens.normal) {
        res.answer = 'YCAM not ready';
      }
      else {
        res.answer = await sens.cset(obj);
      }
      return Promise.resolve(true);
    case 'pset':
      if (!sens.normal) {
        res.answer = 'YCAM not ready';
      }
      else {
        res.answer = await sens.pset(obj);
      }
      return Promise.resolve(true);
    case 'stat': // <--------sensor (maybe YCAM) status query
      return new Promise((resolve) => {
        res.answer = JSON.stringify(sens.stat());
        resolve(true);
      });
    case 'view':
      return new Promise((resolve) => {
        try {
          vue_N = parseInt(cmds[0]);
        }
        catch(err) {
          vue_N = 0;
        }
        image_L.view(vue_N);
        image_R.view(vue_N);
        resolve(true);
      });
    }
  });
});
