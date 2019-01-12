const execSync = require('child_process').execSync;

let resolution='640x480';
let NS='';
let tab='';
let aopt='';
let adds=null;

exports.dump=function(){
  if(arguments.length>0) resolution=arguments[0];
  if(arguments.length>1) NS=arguments[1];
/*  if(arguments.length>2){
      adds=arguments[2];
      aopt='--address='+adds;
    }*/
  if(NS.length>0){
    tab='  ';
    NS=NS+': \n';
  }
  const xmlstr = execSync('arv-tool-0.4 '+aopt+' genicam | tail -n +2').toString();
  let lines = xmlstr.split(/\n/);
  const yoods = '==YOODS==';
  let vga_yamlstr = NS;
  let sxga_yamlstr = NS;
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
  const IDstr = execSync('arv-tool-0.4').toString().trim();
  return {id:IDstr,yaml:yamlstr};
}
