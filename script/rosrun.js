const EventEmitter = require('events').EventEmitter;

let popen = require('child_process');

popen.run=function(node, ns, eo) {
  let ev=eo;
  if(arguments.length<3){
    ev=new EventEmitter();
    ev.on('rerun',function(){
      popen.run(node, ns, ev);
    });
  }
  ev.running=null;
  let env=process.env;
  env=Object.assign(env, { ROS_NAMESPACE: ns });
  let proc=popen.exec('rosrun ' + node, { env:process.env, detached: true });
  ev.running = proc;
  let arg=node.split(' ');
  arg.push('');
  let start_timer=setTimeout(function() {
    start_timer=null;
    ev.emit('start'); // rosrun success
  }, 3000);
  proc.stdout.on('data', function(data) {
    console.log('rosrun:stdout:' + data);
    ev.emit('cout', data);
  });
  proc.stderr.on('data', function(data) {
    console.log('rosrun:stderr:' + data);
    ev.emit('cerr', data);
  });
  proc.on('close', function(code) {
    console.log('rosrun closed:' + code);
    delete proc;
    if (start_timer==null) ev.emit('stop');
    else{
      setTimeout(function(){
        ev.emit('rerun');
        clearTimeout(start_timer);
      },1000);
    }
    ev.running = null;
  });
  return ev;
}

module.exports = popen;
