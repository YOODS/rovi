const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const EventEmitter = require('events').EventEmitter;

exports.assign=function(sens){
  sens.fps=1;
  sens.reqL_=0;
  sens.reqR_=0;
  sens.toutL_=0;
  sens.toutR_=0;
  sens.on('shutdown', async function() {
    sens.scanStop();
  });
  sens.on('left', async function(img) {
    if(sens.reqL_>0) sens.reqL_--;
    if(sens.reqL_==0) sens.emit('syncL');
  });
  sens.on('right', async function(img) {
    if(sens.reqR_>0) sens.reqR_--;
    if(sens.reqR_==0) sens.emit('syncR');
  });
  sens.syncL=async function(tmo){
    return new Promise(function(resolve){
      setTimeout(function(){  sens.reqL_=0; sens.removeAllListeners('syncL'); resolve(true);},tmo);
      sens.once('syncL',function(){ resolve(true);});
    });
  }
  sens.syncR=async function(tmo){
    return new Promise(function(resolve){
      setTimeout(function(){ sens.reqR_=0; sens.removeAllListeners('syncR'); resolve(true);},tmo);
      sens.once('syncR',function(){ resolve(true);});
    });
  }
  sens.streaming=null;
  sens.streamingQ_=null;
  sens.scanStart=function(tmd){
    if(sens.streaming!=null) return;
    if(sens.streamingQ_!=null) clearTimeout(sens.streamingQ_);
    let tm=arguments.length==0? 1:tmd;
    sens.streamingQ_=setTimeout(function(){
      sens.reqL_=sens.reqR_=0;
      sens.streamingQ_=null;
      sens.scanDo_();
    },tm);
  }
  sens.scanDo_=function(){
    if(sens.streaming!=null) return;
    if(sens.reqL_>2) sens.toutL_++;
    else sens.toutL_=0;
    if(sens.reqR_>2) sens.toutR_++;
    else sens.toutR_=0;
    if(sens.toutL_>2 || sens.toutR_>2){
      sens.emit('timeout');
      sens.streaming=null;
      return;
    }
    if(sens.device.pstat){
      sens.emit('trigger');
      sens.reqL_++;
      sens.reqR_++;
    }
    sens.streaming=setTimeout(function(){
      sens.streaming=null;
      sens.scanDo_();
    },Math.floor(1000/sens.fps));
  }
  sens.scanStop=function(tmo){
    if(sens.streamingQ_!=null){
      clearTimeout(sens.streamingQ_);
      return Promise.resolve(true);
    }
    if(sens.streaming!=null) clearTimeout(sens.streaming);
    sens.streaming=null;
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
