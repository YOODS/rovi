#!/usr/bin/env node

const ros = require('rosnodejs');
const net=require('net');

setImmediate(async function(){
//Socket
  const server = net.createServer(function(conn){
    console.log('r_socket_log connection established.');

    let msg='';
    conn.on('data', function(data){
      msg+=data.toString();
      console.log('[R-LOG] msg='+msg);
      msg='';
    });
    conn.on('close', function(){
      console.log('r_socket_log connection closed.');
    });
  }).listen(10003);
});
