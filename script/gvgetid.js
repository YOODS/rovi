#!/usr/bin/env node

const execSync = require('child_process').execSync;

let adds=null;
let arver=6;

if(process.argv.length>2){
  adds=process.argv[2];
}
let ids;
try{
  ids=execSync('arv-tool-0.6').toString();
}
catch(err){
  ids=execSync('arv-tool-0.4').toString();
  arver=4;
}
let idary=ids.split('\n');
if(arver==6){
  let id=null;
  if(adds==null){
    id=idary[0];
  }
  else{
    idary.forEach(function(s){
     if(s.indexOf(adds)>0) id=s;
    });
  }
  if(id!=null){
    adds=id.replace(/.*\(/,'');
    adds=adds.replace(/\).*/,'').trim();
    id=id.replace(/\(.*\)/,'').trim()
    console.log(id+'@@@'+adds);
  }
  else console.log('');
}
else if(arver==4){
 console.log(idary[0].trim());
}
