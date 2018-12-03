const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const EventEmitter = require('events').EventEmitter;

exports.assign=function(sens){
  sens.fps=1;
  sens.reqL_=0;
  sens.reqR_=0;
  sens.on('wake', async function(){
    setTimeout(sens.scanOn,3000);
  });
  sens.on('shutdown', async function() {
    sens.scanOff();
  });
  sens.on('left', async function(img) {
    if(sens.reqL_>0) sens.reqL_--;
    if(sens.reqL_==0){
      sens.emit('syncL');
      sens.reqL_=0;
    }
  });
  sens.on('right', async function(img) {
    if(sens.reqR_>0) sens.reqR_--;
    if(sens.reqR_==0){
      sens.emit('syncR');
      sens.reqR_=0;
    }
  });
  sens.syncL=async function(tmo){
    return new Promise(function(resolve){
      setTimeout(function(){  sens.reqL_=0; resolve(true);},tmo);
      sens.once('syncL',function(){ resolve(true);});
    });
  }
  sens.syncR=async function(tmo){
    return new Promise(function(resolve){
      setTimeout(function(){ sens.reqR_=0; resolve(true);},tmo);
      sens.once('syncR',function(){ resolve(true);});
    });
  }
  sens.scanID=null;
  sens.scanOn=function(){
    if(sens.scanID!=null) return;
    sens.emit('trigger');
    sens.reqL_++;
    sens.reqR_++;
    sens.scanID=setTimeout(function(){
      sens.scanID=null;
      sens.scanOn();
    },Math.floor(1000/sens.fps));
  }
  sens.scanOff=function(tmo){
    if(sens.scanID!=null) clearTimeout(sens.scanID);
    sens.scanID=null;
    if(sens.reqL_>0 && sens.reqR_>0){
ros.log.info('Stop streaming both');
      return Promise.all([sens.syncL(tmo),sens.syncR(tmo)]);
    }
    else if(sens.reqL_>0){
ros.log.info('Stop streaming left');
      return sens.syncL(tmo);
    }
    else if(sens.reqR_>0){
ros.log.info('Stop streaming right');
      return sens.syncR(tmo);
    }
    else{
ros.log.info('Stop streaming neither');
      return Promise.resolve(true);
    }
  }
  return sens;
}
