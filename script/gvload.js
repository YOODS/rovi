#!/usr/bin/env node

const execSync=require('child_process').execSync;
const yaml=require('./gvdump.js');
let vmod='vga';
if(process.argv.length>2) vmod=process.argv[2];

let contents=process.argv.length>3? yaml.dump(vmod,process.argv[3]):yaml.dump(vmod);

try{
  execSync('rosparam load -',{input:contents});
}
catch(err){
  console.log('gvload::rosparam load error');
}
process.exit(0);
