const execSync = require('child_process').execSync;

let resolution='vga';
let tab='';
let aopt='';
let adds=null;
let arver=6;

exports.dump=function(){
  if(arguments.length>0) resolution=arguments[0];
  if(arguments.length>1){
      adds=arguments[1];
      aopt='--address='+adds;
  }
  let xmlstr;
  try{
    xmlstr=execSync('arv-tool-0.6 '+aopt+' genicam | tail -n +2').toString();
  }
  catch(err){
    arver=4;
    xmlstr=execSync('arv-tool-0.4 genicam | tail -n +2').toString();
  }
  if(xmlstr.length==0){
    return {id:'No camera found'};
  }

  let lines = xmlstr.split(/\n/);
  const yoods = '==YOODS==';
  let vga_yamlstr = '';
  let sxga_yamlstr = '';
  let curkey = '';

  for (let li = 0; li < lines.length; li++) {
    if (lines[li].startsWith(yoods)) {
      const key = lines[li].slice(yoods.length);
      if (key === 'start') {
      // ignore
      }
      else if (key === 'vga') {
        curkey = 'vga';
      }
      else if (key === 'sxga') {
        curkey= 'sxga';
      }
      else if (key === 'end') {
        break;
      }
      else {
        console.error(yoods + ' UNKNOWN key...[' + key + ']');
        process.exit(102);
      }
    }
    else {
      if (curkey === 'vga') {
        vga_yamlstr += tab + lines[li] + '\n';
      }
      else if (curkey === 'sxga') {
        sxga_yamlstr += tab + lines[li] + '\n';
      }
    }
  }

  let yamlstr = '';
  if (resolution == 'vga') {
    yamlstr = vga_yamlstr;
  }
  else if (resolution == 'sxga') {
    yamlstr = sxga_yamlstr;
  }
  if(arver==6){
    let IDstr = execSync('arv-tool-0.6').toString().trim();
    IDstr=IDstr.replace(/\(.*\)/,'');
    return {id:IDstr,yaml:yamlstr};
  }
  else if(arver==4){
    let IDstr = execSync('arv-tool-0.4').toString().trim();
    return {id:IDstr,yaml:yamlstr};
  }
  else{
    return {id:'No aravis tools'};
  }
}
