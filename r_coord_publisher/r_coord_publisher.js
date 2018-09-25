#!/usr/bin/env node

const ros = require('rosnodejs');
const geometry_msgs = ros.require('geometry_msgs').msg;
const sensor_msgs = ros.require('sensor_msgs').msg;
const net = require('net');

const ns = '/robot';

let seq = 0;

const jnt = new sensor_msgs.JointState();

function startPubJs(pubjs) {
  // init
  for (let i = 0; i < 6; i++) {
    jnt.name[i] = 'joint' + (i + 1);
    jnt.position[i] = 0.0;
  }

  // loop
  setInterval(function() {
    seq++;
    jnt.header.seq = seq;
    jnt.header.stamp = ros.Time.now();
/*
    console.log('seq=' + jnt.header.seq);
    console.log('stamp.secs=' + jnt.header.stamp.secs);
    console.log('stamp.nsecs=' + jnt.header.stamp.nsecs);
    console.log('frame_id=' + jnt.header.frame_id);
*/
    pubjs.publish(jnt);
  }, 100);
}

function toCoords(data) {
// data format is 'P1[NNNNNNNNNN.NNN](X,Y,Z,A,B,C)(F1,F2)(J1,J2,J3,J4,J5,J6)\n'.
  const str = data.toString();
  const ary = str.replace(/\)\(/g, '],[').replace(/P1.*\(/, '[').replace(/\)/, ']').replace(/\+/g, '');
//  console.log('ary={' + ary + '}');
  let coords = [];
  try {
    coords = JSON.parse('[' + ary + ']');
  }
  catch(e) {
    console.log("error " + e);
  }
  return coords;
}

setImmediate(async function() {
  const rosNode = await ros.initNode(ns + '/r_coord_publisher');

//Publisher
  const pub_euler = rosNode.advertise(ns + '/euler', geometry_msgs.Transform);
  const pub_js = rosNode.advertise(ns + '/joint_states', sensor_msgs.JointState);
  startPubJs(pub_js);

//Socket
  const server = net.createServer(function(conn) {
    console.log('connection established');
    conn.on('data', function(data) {
      const coords = toCoords(data);
      if (coords.length < 3) {
        console.log('r_coord error:' + coords);
        return;
      }
      if (coords[0].length == 6) {
        let tfe = new geometry_msgs.Transform();
        tfe.translation.x = coords[0][0];
        tfe.translation.y = coords[0][1];
        tfe.translation.z = coords[0][2];
        tfe.rotation.x = coords[0][3];
        tfe.rotation.y = coords[0][4];
        tfe.rotation.z = coords[0][5];
        pub_euler.publish(tfe);
      }
      if (coords[2].length == 6) {
        for (let i = 0; i < 6; i++) {
          jnt.position[i] = coords[2][i];
        }
      }
    });
    conn.on('close', function() {
      console.log('connection closed');
    });
  }).listen(10002);

});
