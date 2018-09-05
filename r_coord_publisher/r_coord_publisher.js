#!/usr/bin/env node

const ros = require('rosnodejs');
const geometry_msgs = ros.require('geometry_msgs').msg;
const sensor_msgs = ros.require('sensor_msgs').msg;
const net = require('net');

const ns = '/robot';

function toTfEuler(data) {
// data format is 'P1******(X,Y,Z,A,B,C)(F1,F2)(J1,J2,J3,J4,J5,J6)\n'.
  const str=data.toString();
  const ary=str.replace(/\)\(/g,'],[').replace(/P1.*\(/,'[').replace(/\)/,']').replace(/\+/g,'');
  console.log(ary);
  let coords=[];
  try{
    coords=JSON.parse('['+ary+']');
  }
  catch(e){
    console.log("error "+e);
  }
  return coords;
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
      const coords = toTfEuler(data);
      if(coords.length<3){
        console.log('r_coord error:'+coords);
        return;
      }
      if(coords[0].length==6){
        let tfe = new geometry_msgs.Transform();
        tfe.translation.x = coords[0][0];
        tfe.translation.y = coords[0][1];
        tfe.translation.z = coords[0][2];
        tfe.rotation.x = coords[0][3];
        tfe.rotation.y = coords[0][4];
        tfe.rotation.z = coords[0][5];
        pub_euler.publish(tfe);
      }
      if(coords[2].length==6){
        let jnt = new sensor_msgs.JointState();
        jnt.name='RV4FL';
        coords[2].forEach(function(c){
          jnt.position.push(c);
          jnt.velocity.push(0);
          jnt.effort.push(0);
        });
//        pub_js.publish(jnt);
      }
    });
    conn.on('close', function() {
      console.log('connection closed');
    });
  }).listen(10002);

});
