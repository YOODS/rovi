#!/usr/bin/env node

const ros = require('rosnodejs');
const geometry_msgs = ros.require('geometry_msgs').msg;
const sensor_msgs = ros.require('sensor_msgs').msg;
const net = require('net');

const ns = '/robot';

function toTfEuler(data) {
  // data format is '(X,Y,Z,A,B,C)(F1,F2)\n'.
  // We use X, Y, Z, A, B, and C.
  const xyzabc = data.toString().split(',', 6);

  if (xyzabc.length !== 6) {
    return;
  }
  else {
    let tfe = new geometry_msgs.Transform();
    tfe.translation.x = parseFloat(xyzabc[0].slice(1)); // discard '('
    tfe.translation.y = parseFloat(xyzabc[1]);
    tfe.translation.z = parseFloat(xyzabc[2]);
    tfe.rotation.x = parseFloat(xyzabc[3]);
    tfe.rotation.y = parseFloat(xyzabc[4]);
    tfe.rotation.z = parseFloat(xyzabc[5]); // parseFloat() discards ')(F1')
    return tfe;
  }
}

setImmediate(async function() {
  const rosNode = await ros.initNode(ns + '/r_coord_publisher');

//Publisher
  const pub_js = rosNode.advertise(ns + '/joint_states', sensor_msgs.JointState);
  const pub_euler = rosNode.advertise(ns + '/euler', geometry_msgs.Transform);

//Socket
  const server = net.createServer(function(conn) {
    console.log('connection established');
    conn.on('data', function(data) {
      //console.log('got data[' + data + ']');
      const tf_euler = toTfEuler(data);
      if (tf_euler) {
        //console.log('publish... tf_euler.translation.x=' + tf_euler.translation.x);
        pub_euler.publish(tf_euler);
      }
    });
    conn.on('close', function() {
      console.log('connection closed');
    });
  }).listen(7373);

});
