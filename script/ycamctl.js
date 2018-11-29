#!/usr/bin/env node

const NSycamctrl = '/rovi/ycam_ctrl';
const NSpsgenpc = '/rovi/pshift_genpc';
const NSX1 = '/rovi/X1';
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
const ImageSwitcher = require('./image_switcher.js');
const Notifier = require('./notifier.js');

function sleep(msec){return new Promise(function(resolve){setTimeout(function(){resolve(true);},msec);});}


setImmediate(async function() {
  const rosNode = await ros.initNode(NSycamctrl);
  const image_L = new ImageSwitcher(rosNode, NScamL);
  const image_R = new ImageSwitcher(rosNode, NScamR);
  const pub_stat = rosNode.advertise(NSycamctrl + '/stat', std_msgs.Bool);
  let vue_N = 0;
  const genpc = rosNode.serviceClient(NSrovi + '/genpc', rovi_srvs.GenPC, { persist: true });
  if (!await rosNode.waitForService(genpc.getService(), 2000)) {
    ros.log.error('genpc service not available');
    return;
  }
  let param={
    camps: new Notifier(rosNode,NSpsgenpc + '/camera'),//Genpc mode camera params
    camlv: new Notifier(rosNode,NSlive + '/camera'),  //Live mode camera params
    proj: new Notifier(rosNode,NSpsgenpc + '/projector') //projector params
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

  sens.scanID=null;
  sens.scanOn=function(){
    if(sens.scanID!=null) return;
    param.proj.push({Go:-1});
    sens.scanID=setTimeout(function(){
      sens.scanID=null;
      sens.scanOn();
    },Math.floor(1000/param.camlv.objs.AcquisitionFrameRate));
  }
  sens.scanOff=function(){
    if(sens.scanID!=null) clearTimeout(sens.scanID);
    sens.scanID=null;
  }

  let sensEv;
  switch (sensName) {
  case 'ycam3':
  case 'ycam3s':
    sensEv = sens.open(rosNode, NSrovi);
    break;
  case 'ycam1h':
    sensEv = sens.open(rosNode,NScamL,image_L.ID,NScamR,image_R.ID, param.proj.objs.Url, param.proj.objs.Port);
    break;

  }
  sensEv.pstat_=false;
  sensEv.on('stat', function(s) {
    let f = new std_msgs.Bool();
    f.data = s;
    pub_stat.publish(f);
    if (sensEv.pstat_ != s) {
      ros.log.warn('YCAM stat becomes ' + s);
    }
    sensEv.pstat_ = s;
  });
  sensEv.on('wake', async function() {
    console.log('ycam wake');
    for(let n in param) await param[n].start();
    param.camlv.push({TriggerMode:'On'});
    param.proj.push({Mode:1});//--- let projector pattern to be phase shift
    setTimeout(sens.scanOn,3000);
    ros.log.warn('NOW ALL READY');
  });
  sensEv.on('shutdown', async function() {
    console.log('ycam down');
    sens.scanOff();
    for(let n in param) param[n].reset();
  });
  sensEv.on('left', async function(img) {
    image_L.emit(img);
  });
  sensEv.on('right', async function(img) {
    image_R.emit(img);
  });



// ---------Definition of services
  let pserror=0,psthres=0;
  let psgenpc = function(req,res){
    if(pserror<0) return false;
    if (!sens.normal) {
      ros.log.warn(res.message = 'YCAM not ready');
      res.success = false;
      return true;
    }
    return new Promise(async (resolve) => {
      let wdt = setTimeout(async function() { //---start watch dog
        image_L.cancel();
        image_R.cancel();
        sens.pblock=false;//---release pset method
        sens.scanOn();
        const errmsg = 'pshift_genpc timed out AAA';
        ros.log.error(errmsg);
        res.success = false;
        res.message = errmsg;
        pserror++;
        resolve(true);
      }, param.proj.objs.Interval*13*2 + 1000);

      sens.scanOff();
//      sens.pblock=true;//---block pset method
      await sens.cset(param.camps.objs); //---overwrites genpc camera params
      await sleep(10);
      setTimeout(function(){
        sens.pblock=false;//---release pset method
        sens.pset({ 'Go': 2 }); //---projector starts after 100ms
      },100);
      let imgs=await Promise.all([image_L.store(13,100),image_R.store(13,100)]);//---after 100msec, try to store 13 images
      clearTimeout(wdt);
      let gpreq = new rovi_srvs.GenPC.Request();
      gpreq.imgL = imgs[0];
      gpreq.imgR = imgs[1];
      let gpres;
      try {
        gpres = await genpc.call(gpreq);
        res.message = imgs[0].length + ' images scan compelete. Generated PointCloud Count=' + gpres.pc_cnt;
        res.success = true;
      }
      catch(err) {
        res.message = 'genpc failed';
        res.success = false;
      }
      let ts=Math.floor(gpres.pc_cnt*0.001)+1;
      setTimeout(function(){ sens.scanOn();},ts);//---after "ts" msec, back to live mode
      param.camlv.push(param.camlv.diff(param.camps.objs));//---restore overwritten camera params
      image_L.view(vue_N);
      image_R.view(vue_N);
      image_L.cancel(ts);//---after "ts" msec, back to thru mode
      image_R.cancel(ts);
      if(gpres.pc_cnt<psthres) pserror=-101;
      ros.log.info('Time to publish pc '+ts+' ms, errors:'+pserror);
      resolve(true);
    });
  }
  const svc_do = rosNode.advertiseService(NSpsgenpc, std_srvs.Trigger, psgenpc);
  const sub_do = rosNode.subscribe(NSX1, std_msgs.Bool,async function(){
    let req=new std_srvs.Trigger.Request();
    let res=new std_srvs.Trigger.Response();
    await psgenpc(req,res);
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
    case 'cstop':
      sens.scanOff();
      return Promise.resolve(true);
    case 'cstart':
      sens.scanOn();
      return Promise.resolve(true);
    }
  });
});
