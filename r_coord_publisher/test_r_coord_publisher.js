#!/usr/bin/env node

const net = require('net');

var client = new net.Socket();

client.connect(7373, '127.0.0.1', function() {
  console.log('connected to server');

  client.write('(0.2,0.9,1.88,-2,139.8875,-0.0092)(123,456)\n');
  setTimeout(function() {
    client.write('(-999.9,-0.9,-1.88,2,139.8875,0.0092)(123,456)\n');
  }, 10);

//  client.write('(4.2,0.9,1.88,-2,139.8875,-0.0092)(123,456)\n(100.2,-5.678,-3.45,0.987,2)(1,2)\n');
//  client.destroy();
});
