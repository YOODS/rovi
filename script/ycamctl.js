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
function add_sendmsg(pub){
  pub.sendmsg=function(s){
    let m=new std_msgs.String();
    m.data=s;
    pub.publish(m);
  }
}

setImmediate(async function() {
  const rosNode = await ros.initNode(NSycamctrl);
  const image_L = new ImageSwitcher(rosNode, NScamL);
  const image_R = new ImageSwitcher(rosNode, NScamR);
//---------publisher and subscriber 1/2
  const pub_stat = rosNode.advertise(NSrovi + '/stat', std_msgs.Bool);
  const pub_error = rosNode.advertise('/error', std_msgs.String);
  add_sendmsg(pub_error);
  const pub_info = rosNode.advertise('/message', std_msgs.String);
  add_sendmsg(pub_info);
  const pub_pcount=rosNode.advertise(NSrovi+'/pcount',std_msgs.Int32);
  const pub_Y1=rosNode.advertise(NSrovi+'/Y1',std_msgs.Bool);
  const genpc=rosNode.serviceClient(NSgenpc, rovi_srvs.GenPC, { persist: true });
  if (!await rosNode.waitForService(genpc.getService(), 2000)) {
    ros.log.error('genpc service not available');
    return;
  }
  const param_camnode=await rosNode.getParam(NSrovi + '/camera');
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
//    if(key=='Mode'){
    if(key!='Go'){
      if(sensEv.streaming){
        await sensEv.scanStop(1000);
        sensEv.lit=false;
        sensEv.scanStart(1000);
      }
    }
    await sens.pset(obj);
  });
  let sensEv;
  switch(sensName){
  case 'ycam3':
  case 'ycam3s':
    sensEv = await sens.open(rosNode, NSrovi, param_camnode);
    break;
  case 'ycam1s':
    sensEv = await sens.open(rosNode,NScamL, NScamR, param.proj.objs.Url, param.proj.objs.Port);
    break;
  }
  sensEv=SensControl.assign(sensEv);
  sensEv.on('wake', async function() {
    for(let n in param) await param[n].start();
    param.camlv.raise({TriggerMode:'On'});
    param.proj.raise({Mode:1});//--- let 13 pattern mode
    ros.log.warn('NOW ALL READY ');
    pub_info.sendmsg('YCAM ready');
    sensEv.lit=false;
    sensEv.scanStart(3000);
  });
  sensEv.on('stat', function(f){
    let m=new std_msgs.Bool();
    m.data=f;
    pub_stat.publish(m);
  });
  sensEv.on('shutdown', async function(){
    ros.log.info('ycam down '+sens.cstat+' '+sens.pstat);
    for(let n in param) param[n].reset();
    rosNode.setParam(NSrovi+'/camera',param_camnode);
    pub_error.sendmsg('YCAM disconected');
  });
  sensEv.on('left', async function(img,ts){
    image_L.emit(img,ts,sensEv.lit);
  });
  sensEv.on('right', async function(img,ts){
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
    pub_error.sendmsg('Image streaming timeout');
    sens.kill();
  });

//---------Definition of services
  let pslock=true;
  sensEv.on('trigger',function(){ pslock=false; });
  sensEv.on('shutdown',function(){ pslock=true; });
  let ps2live = function(tp){ //---after "tp" msec, back to live mode
    setTimeout(function(){
      sensEv.scanStart();
      image_L.thru();
      image_R.thru();
      pslock=false;
    },tp);
    param.camlv.raise(param.camps.diff(param.camlv.objs));//---restore overwritten camera params
  }
  let psgenpc = function(req,res){
    if(!sens.normal){
      ros.log.warn(res.message='YCAM not ready');
      res.success = false;
      return true;
    }
    if(pslock){
      ros.log.warn(res.message='genpc busy');
      res.success = false;
      return true;
    }
    return new Promise(async (resolve) => {
      pslock=true;
      await sensEv.scanStop(1000); //---wait stopping stream with 1000ms timeout
      ros.log.info('Streaming stopped');
      await sens.cset(param.camlv.diff(await param.camps.param())); //---overwrites genpc camera params
      let dpj=param.proj.diff(await param.proj.param());
      ros.log.info("Proj param "+JSON.stringify(dpj));
      await sens.pset(dpj); //---overwrites genpc projector params
      let wdt=setTimeout(async function() { //---start watch dog
        ps2live(1000);
        const errmsg = 'pshift_genpc timed out';
        pub_error.sendmsg(errmsg);
        res.success = false;
        res.message = errmsg;
        pub_Y1.publish(new std_msgs.Bool());
        param.proj.raise({Mode:1});//---reload 13 pattern
        resolve(true);
      }, param.proj.objs.Interval*13 + 2000);
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
      let imgs;
      try{
        imgs=await Promise.all([image_L.store(13),image_R.store(13)]); //---switch to "store" mode
      }
      catch(err){
        const msg="image_switcher::exception";
        ps2live(1000);
        ros.log.error(msg);
        pub_error.sendmsg(msg);
        res.success = false;
        res.message = msg;
        resolve(true);
        pub_Y1.publish(new std_msgs.Bool());
        return;
      }     
      clearTimeout(wdt);
      let gpreq = new rovi_srvs.GenPC.Request();
      gpreq.imgL = imgs[0];
      gpreq.imgR = imgs[1];
      let gpres;
      try {
        ros.log.info("call genpc");
        gpres = await genpc.call(gpreq);
        ros.log.info("ret genpc");
        res.message = imgs[0].length + ' images scan complete. Generated PointCloud Count=' + gpres.pc_cnt;
        res.success = true;
      }
      catch(err) {
        res.message = 'genpc failed';
        res.success = false;
      }
      let pcount=new std_msgs.Int32();
      pcount.data=gpres.pc_cnt;
      pub_pcount.publish(pcount);
      let tp=Math.floor(gpres.pc_cnt*0.0001)+10;
      ps2live(1000);
      ros.log.info('Time to publish pc '+tp+' ms');
      let finish=new std_msgs.Bool();
      finish.data=res.success;
      pub_Y1.publish(finish);
      resolve(true);
    });
  }

//---------publisher and subscriber 2/2
  const svc_X1=rosNode.advertiseService(NSps, std_srvs.Trigger, psgenpc);
  const sub_X1=rosNode.subscribe(NSrovi+'/X1',std_msgs.Bool,async function(){
    if (!sens.normal){
      pub_error.sendmsg('request cancelled due to YCAM status');
      pub_Y1.publish(new std_msgs.Bool());
      return;
    }
    let req=new std_srvs.Trigger.Request();
    let res=new std_srvs.Trigger.Response();
    let ret=psgenpc(req,res);
//  if(typeof(ret)=='boolean'){ //request refused
//    pub_Y1.publish(new std_msgs.Bool());
//  }
  });
  const svc_reset = rosNode.subscribe(NSrovi+'/reset',std_msgs.Bool,async function(){
    await sens.reset();
  });
});
