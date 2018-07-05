const ros = require('rosnodejs');
const EventEmitter = require('events').EventEmitter;

function calcHash(text) {
    'use strict';

    var hash = 5381,
        index = text.length;

    while (index) {
        hash = (hash * 33) ^ text.charCodeAt(--index);
    }

    return hash >>> 0;
}

class Notifier extends EventEmitter{
  constructor(ros,ns){
    super();
    this.ros=ros;
    this.ns=ns;
    this.params={};
    this.tid=null;
    this.enable=false;
  }
  check(param){
    let keys=Object.keys(param);
    if(this.enable){
      if(keys.length>0){
        let key=keys[0];
        let val=param[key];
        let hash=calcHash(JSON.stringify(val));
        if(! this.params.hasOwnProperty(key) || this.params[key]!=hash){
          this.params[key]=hash;
          this.emit('change',key,val);
        }
        delete param[key];
        const who=this;
        setImmediate(function(){ who.check(param);});
      }
      else{
        const who=this;
        this.tid=setTimeout(async function(){
          who.tid=null;
          let param;
          try{
            param=await who.ros.getParam(who.ns);
          }
          catch(err){
            console.log('getParam err:'+who.ns);
            who.stop();
          }
          who.check(param);
        },1000);
      }
    }
  }
  start(){
    if(!this.enable){
      this.enable=true;
      this.emit('start');
      this.check({}); //start scanning
    }
  }
  stop(){
    if(this.enable){
      this.enable=false;
      if(this.tid!=null) clearTimeout(this.tid);
      this.emit('stop');
    }
  }
}

module.exports=Notifier
