#!/usr/bin/env node

console.log('arvc '+process.argv.length);
const execSync=require('child_process').execSync;
const yaml=require('./gvdump.js');
let contents=process.argv.length>3? yaml.dump(process.argv[2],'rovi',process.argv[3]):yaml.dump(process.argv[2],'rovi');

execSync('rosparam set /rovi/camera/ID '+'"'+contents.id+'"');
try{
  execSync('rosparam load -',{input:contents.yaml});
}
catch(err){
  console.log('gvload::rosparam load error');
}
if(contents.id.toUpperCase().startsWith('NO DEVICE')){
  console.log('gvload::no xml data');
  process.exit(404);
}
else{
  process.exit(0);
}
