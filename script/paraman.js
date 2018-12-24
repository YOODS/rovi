#!/usr/bin/env node

const ros = require('rosnodejs');
const std_msgs = ros.require('std_msgs').msg;
const Notifier = require('./notifier.js');

setImmediate(async function() {
  if(process.argv.length<4){
    console.log('usage: paraman.js <node of parameter> <topic to inform changing>...');
    process.exit(1)
  }
  const rosNode=await ros.initNode('paraman_'+ros.Time.now().secs+'.'+ros.Time.now().nsecs.toString().substr(0,3));
  const notif=new Notifier(rosNode,process.argv[2])
  let pubs=[];
  for(let i=3;i<process.argv.length;i++){
    let p=process.argv[i];
    if(p.indexOf(':=')<0){
      console.log(p.indexOf(':=')+' '+p);
      pubs.push(rosNode.advertise(p,std_msgs.String,{queueSize:32}));
    }
  }
  notif.on('change',function(key,val){
    let msg=new std_msgs.String();
    let v=Number(val);
    if(Number.isNaN(v)) v='"'+val+'"';
    msg.data=key+'='+v;
    pubs.forEach(function(p){
      p.publish(msg);
    });
  });
  setTimeout(function(){
    notif.start();
  },2000);
});
