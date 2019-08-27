#!/usr/bin/env node

const NSrovi = process.env.ROS_NAMESPACE;
const NSycamctrl = NSrovi+'/ycam_ctrl';
const NSps = NSrovi+'/pshift_genpc';
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
const rovi_msgs = ros.require('rovi').msg;
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
  const pub_stat = rosNode.advertise(NSrovi + '/stat', std_msgs.Bool);
  const pub_error = rosNode.advertise(NSrovi + '/error', std_msgs.String);
  const errormsg=function(msg){
    let err=new std_msgs.String();
    err.data=msg;
    pub_error.publish(err);
    ros.log.info('Error:'+err.data);
  }
  const pub_pcount=rosNode.advertise(NSrovi+'/pcount',std_msgs.Int32);
  const genpc=rosNode.serviceClient(NSgenpc, rovi_srvs.GenPC, { persist: true });
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
  sensEv.wakeup_timer=null;
  sensEv.on('wake', async function() {
    ros.log.info('ycam wake');
    for(let n in param) await param[n].start();
    param.camlv.raise({TriggerMode:'On'});
    param.proj.raise({Mode:1});//--- let 13 pattern mode
//    param.proj.raise({Mode:2});//--- let projector pattern to max brightness
    ros.log.warn('NOW ALL READY ');
    errormsg('YCAM ready');
    sensEv.lit=false;
    sensEv.wakeup_timer=setTimeout(function(){
      sensEv.scanStart();
      sensEv.wakeup_timer=null;
    },3000);
  });
  sensEv.on('stat', function(f){
    let m=new std_msgs.Bool();
    m.data=f;
    pub_stat.publish(m);
  });
  sensEv.on('shutdown', async function() {
    ros.log.info('ycam down '+sens.cstat+' '+sens.pstat);
    for(let n in param) param[n].reset();
    errormsg('YCAM disconected');
  });
  sensEv.on('left', async function(img,ts) {
    image_L.emit(img,ts,sensEv.lit);
  });
  sensEv.on('right', async function(img,ts) {
    image_R.emit(img,ts,sensEv.lit);
  });
  sensEv.on('trigger', async function() {
    if(param.proj.objs.Mode==1){
      sensEv.lit=true;
      param.proj.raise({Go:-1});
    }
    else{
      sensEv.lit=!sensEv.lit;
      if(sensEv.lit) param.proj.raise({Go:1});
      else param.proj.raise({Go:-1});
    }
    sensEv.fps=param.camlv.objs.SoftwareTriggerRate;
  });
  sensEv.on('timeout', async function() {
    ros.log.error('Image streaming timeout');
    sens.kill();
  });

// ---------Definition of services
  let psthres=100;
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
    if (!sens.normal) {
      ros.log.warn(res.message = 'YCAM not ready');
      res.success = false;
      return true;
    }
    if(sensEv.wakeup_timer!=null){
      clearTimeout(sensEv.wakeup_timer);
      sensEv.wakeup_timer=null;
    }
    return new Promise(async (resolve) => {
//      param.proj.raise({Mode:1});//--- let projector pattern to be phase shift
      await sensEv.scanStop(1000); //---wait stopping stream with 1000ms timeout
      ros.log.info('Streaming stopped');
      await sens.cset(param.camps.objs); //---overwrites genpc camera params
      let wdt=setTimeout(async function() { //---start watch dog
        ps2live(1000);
        const errmsg = 'pshift_genpc timed out';
        ros.log.error(errmsg);
        errormsg(errmsg);
        res.success = false;
        res.message = errmsg;
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
      let pcount=new std_msgs.Int32();
      pcount.data=gpres.pc_cnt;
      pub_pcount.publish(pcount);
      let tp=Math.floor(gpres.pc_cnt*0.001)+1;
      ps2live(tp);
      if(gpres.pc_cnt<psthres) errormsg('Points count '+gpres.pc_cnt+' too few');
      ros.log.info('Time to publish pc '+tp+' ms');
      resolve(true);
    });
  }
  const svc_do = rosNode.advertiseService(NSps, std_srvs.Trigger, psgenpc);
  const sub_do = rosNode.subscribe(NSrovi+'/X1',std_msgs.Bool,async function(){
    if (!sens.normal){
      errormsg('Request cancelled due to YCAM status');
      return;
    }
    let req=new std_srvs.Trigger.Request();
    let res=new std_srvs.Trigger.Response();
    await psgenpc(req,res);
  });
  const svc_reset = rosNode.subscribe(NSrovi+'/reset',std_msgs.Bool,async function(){
    param.proj.raise({Reset:1});
  });
});
