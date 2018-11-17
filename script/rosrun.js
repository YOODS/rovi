const EventEmitter = require('events').EventEmitter;

let popen = require('child_process');

popen.run = function(node, ns, eo) {
  let ev = arguments.length < 3 ? new EventEmitter():eo;
  ev.running = false;
  let env = process.env;
  env = Object.assign(env, { ROS_NAMESPACE: ns });
  let proc = popen.exec('rosrun ' + node, { env: process.env });
  let arg=node.split(' ');
  arg.push('');
  let stm = setTimeout(function() {
    ev.running = true;
    ev.emit('start'); // rosrun success
  }, 5000);
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
    if (ev.running) {
      ev.running = false;
      ev.emit('stop');
    }
    else {
      clearTimeout(stm);
    }
    setTimeout(function() {
      popen.run(node, ns, ev);
    }, 3000);
  });
  return ev;
}

module.exports = popen;
