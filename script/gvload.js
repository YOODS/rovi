#!/usr/bin/env node

console.log('arvc '+process.argv.length);
const execSync=require('child_process').execSync;
const yaml=require('./gvdump.js');
let contents=process.argv.length>3? yaml.dump(process.argv[2],process.argv[3]):yaml.dump(process.argv[2]);

if(contents.id.startsWith('No ')){
  console.log('gvload error::'+contents.id);
  process.exit(404);
}

execSync('rosparam set camera/ID '+'"'+contents.id+'"');
try{
  execSync('rosparam load -',{input:contents.yaml});
}
catch(err){
  console.log('gvload::rosparam load error');
}
process.exit(0);
