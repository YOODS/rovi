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
    this.hashes={};
    this.objs={};
    this.tid=null;
    this.enable=false;
    this.init();
  }
  async init(){
    try{
      this.objs=await this.ros.getParam(this.ns);
      return true;
    }
    catch(err){
      console.log('Init Param err:'+this.ns);
      return false;
    }
  }
  raise(param,diff,cb){
    switch(arguments.length){
    case 0:
      param=Object.assign({},this.objs);
    case 1:
      diff=false;
    case 2:
      cb=null;
    }
    let keys=Object.keys(param);
    if(keys.length==0){
      if(cb!=null) setImmediate(function(){
        cb();
      });
      return;
    }
    let key=keys[0];
    let val=param[key];
    let hash=calcHash(JSON.stringify(val));
    if((!diff) || (!this.hashes.hasOwnProperty(key)) || this.hashes[key]!=hash){
      this.hashes[key]=hash;
      this.objs[key]=val;
      this.emit('change',key,val);
    }
    delete param[key];
    const who=this;
    setImmediate(function(){
      who.raise(param,diff,cb);
    });
  }
  async check(param){
    let keys=Object.keys(param);
    if(this.enable){
      if(keys.length>0){
        let who=this;
        this.raise(param,true,function(){
          who.tid=setTimeout(async function(){
            who.tid=null;
            who.check({});
          },1000);
        });
      }
      else{
        let param;
        try{
          param=await this.ros.getParam(this.ns);
          this.check(param);
        }
        catch(err){
          console.log('getParam err:'+this.ns);
          this.stop();
        }
      }
    }
  }
  async start(){
    if(this.enable) return;
    if(! await this.init()) return;
    this.enable=true;
    this.emit('start');
    this.check({});
  }
  stop(){
    if(this.enable){
      this.enable=false;
      if(this.tid!=null) clearTimeout(this.tid);
      this.emit('stop');
    }
  }
  reset(){
    this.stop();
    this.hashes={};
  }
  diff(obj){
    let c={};
    for(let k in obj){
      if (this.objs.hasOwnProperty(k)){
        if(this.objs[k]!=c[k]) c[k]=this.objs[k];
      }
    }
    return c;
  }
}

module.exports=Notifier
