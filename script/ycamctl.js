#!/usr/bin/env node

const NSrovi = process.env.ROS_NAMESPACE;
const NSycamctrl = NSrovi+'/ycam_ctrl';
const NSps = NSrovi+'/pshift_genpc';
const NSX1 = NSrovi+'/X1';
const NSgenpc = NSrovi+'/genpc';
const NScamL = NSrovi+'/left';
const NScamR = NSrovi+'/right';
const NSlive = NSrovi+'/live';
const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const sensName = process.argv.length < 3 ? 'ycam1s' : process.argv[2];
const sens = require('./' + sensName + '.js')
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const rovi_srvs = ros.require('rovi').srv;
const EventEmitter = require('events').EventEmitter;
const jsyaml = require('js-yaml');
const ImageSwitcher = require('./image_switcher.js');
const Notifier = require('./notifier.js');
const SensControl = require('./sens_ctrl.js');

function sleep(msec){return new Promise(function(resolve){setTimeout(function(){resolve(true);},msec);});}


setImmediate(async function() {
  const rosNode = await ros.initNode(NSycamctrl);
  const image_L = new ImageSwitcher(rosNode, NScamL);
  const image_R = new ImageSwitcher(rosNode, NScamR);
  const pub_stat = rosNode.advertise(NSycamctrl + '/stat', std_msgs.Bool);
  const pub_errlog = rosNode.advertise(NSycamctrl + '/errlog', std_msgs.Bool);
  let errlog='';
  let vue_N = 0;
  const genpc = rosNode.serviceClient(NSgenpc, rovi_srvs.GenPC, { persist: true });
  if (!await rosNode.waitForService(genpc.getService(), 2000)) {
    ros.log.error('genpc service not available');
    return;
  }
  let param={
    camps: new Notifier(rosNode,NSps + '/camera'),//Genpc mode camera params
    camlv: new Notifier(rosNode,NSlive + '/camera'),  //Live mode camera params
    proj: new Notifier(rosNode,NSps + '/projector') //Genpc projector params
  };
  param.camlv.on('change',async function(key,val){
    let obj={};
    obj[key]=val;
    await sens.cset(obj);
  });
  param.proj.on('change',async function(key,val){
    let obj={};
    obj[key]=val;
    await sens.pset(obj);
  });

  let sensEv;
  switch(sensName){
  case 'ycam3':
  case 'ycam3s':
    sensEv = await sens.open(rosNode, NSrovi);
    break;
  case 'ycam1s':
    sensEv = await sens.open(rosNode,NScamL, NScamR, param.proj.objs.Url, param.proj.objs.Port);
    break;
  }
  sensEv=SensControl.assign(sensEv);
  sensEv.on('wake', async function() {
    ros.log.info('ycam wake');
    for(let n in param) await param[n].start();
    param.camlv.raise({TriggerMode:'On'});
    param.proj.raise({Mode:1});//--- let 13 pattern mode
//    param.proj.raise({Mode:2});//--- let projector pattern to max brightness
    ros.log.warn('NOW ALL READY');
  });
  sensEv.on('stat', function(f){
    let m=new std_msgs.Bool();
    m.data=f;
    pub_stat.publish(m);
  });
  sensEv.on('shutdown', async function() {
    ros.log.info('ycam down '+sens.cstat+' '+sens.pstat);
    for(let n in param) param[n].reset();
  });
  sensEv.on('left', async function(img,ts) {
    image_L.emit(img,ts);
  });
  sensEv.on('right', async function(img,ts) {
    image_R.emit(img,ts);
  });
  sensEv.on('trigger', async function() {
    switch(param.proj.objs.Mode){
    case 1:
      param.proj.raise({Go:-1});
      break;
    case 2:
      param.proj.raise({Go:1});
      break;
    }
    sensEv.fps=param.camlv.objs.SoftwareTriggerRate;
  });

// ---------Definition of services
  let pserror=0,psthres=0;
  let ps2live = function(tp){ //---after "tp" msec, back to live mode
    setTimeout(function(){
      sensEv.scanStart();
      image_L.thru();
      image_R.thru();
    },tp);
    param.camlv.raise(param.camlv.diff(param.camps.objs));//---restore overwritten camera params
//    param.proj.raise({Mode:2});//--- let projector pattern to max brightness
  }
  let psgenpc = function(req,res){
//    if(pserror<0) return false;
    if (!sens.normal) {
      ros.log.warn(res.message = 'YCAM not ready');
      res.success = false;
      return true;
    }
    return new Promise(async (resolve) => {
//      param.proj.raise({Mode:1});//--- let projector pattern to be phase shift
      await sensEv.scanStop(1000); //---wait stopping stream with 1000ms timeout
      ros.log.info('Streaming stopped');
      await sens.cset(param.camps.objs); //---overwrites genpc camera params
      let wdt=setTimeout(async function() { //---start watch dog
        ps2live(1000);
        const errmsg = 'pshift_genpc timed out AAA';
        ros.log.error(errmsg);
        res.success = false;
        res.message = errmsg;
        pserror--;
        resolve(true);
      }, param.proj.objs.Interval*13 + 1000);
//for monitoring
      let icnt=0;
      image_L.hook.on('store',function(img,t2){
        let t1=img.header.stamp;
        ros.log.info(('00'+icnt.toString(10)).substr(-2)+' '+(t1.nsecs*1e-9+t1.secs)+' '+(t2.nsecs*1e-9+t2.secs));
        icnt++;
      });
//
      ros.log.info('Ready to store');
      setImmediate(function(){ sens.pset({ 'Go': 2 });});  //---projector starts in the next loop
      await sleep(100);
      let imgs=await Promise.all([image_L.store(13),image_R.store(13)]); //---switch to "store" mode
      clearTimeout(wdt);
      let gpreq = new rovi_srvs.GenPC.Request();
      gpreq.imgL = imgs[0];
      gpreq.imgR = imgs[1];
      let gpres;
      try {
        ros.log.info("call genpc");
        gpres = await genpc.call(gpreq);
        ros.log.info("ret genpc");
        res.message = imgs[0].length + ' images scan compelete. Generated PointCloud Count=' + gpres.pc_cnt;
        res.success = true;
      }
      catch(err) {
        res.message = 'genpc failed';
        res.success = false;
      }
      let tp=Math.floor(gpres.pc_cnt*0.001)+1;
      ps2live(tp);
      image_L.view(vue_N);
      image_R.view(vue_N);
      if(gpres.pc_cnt<psthres) pserror=-101;
      ros.log.info('Time to publish pc '+tp+' ms, errors:'+pserror);
      resolve(true);
    });
  }
  const svc_do = rosNode.advertiseService(NSps, std_srvs.Trigger, psgenpc);
  const sub_do = rosNode.subscribe(NSX1, std_msgs.Bool,async function(){
    if (!sens.normal) return;
    let req=new std_srvs.Trigger.Request();
    let res=new std_srvs.Trigger.Response();
    await psgenpc(req,res);
  });
  const svc_reset = rosNode.advertiseService(NSycamctrl + '/reset', std_srvs.Trigger, function(req,res){
    param.proj.raise({Reset:1});
    pserror=0;
    res.message = '';
    res.success = true;
    return true;
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
      res.answer = await sens.cset(obj);
      return Promise.resolve(true);
    case 'pset':
      res.answer = await sens.pset(obj);
      return Promise.resolve(true);
    case 'stat': //---sensor(maybe YCAM) status query
      return new Promise((resolve) => {
        res.answer = JSON.stringify(sens.stat());
        resolve(true);
      });
    case 'view':
      return new Promise((resolve) => {
        try {
          vue_N = Number(cmds[0]);
        }
        catch(err) {
          vue_N = 0;
        }
        image_L.view(vue_N);
        image_R.view(vue_N);
        resolve(true);
      });
    case 'thres':
      pserror=0;
      psthres=Number(cmds[0]);
      return Promise.resolve(true);
    }
  });
});
