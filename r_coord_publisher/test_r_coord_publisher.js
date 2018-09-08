#!/usr/bin/env node

const net = require('net');

var client = new net.Socket();

client.connect(10002, '127.0.0.1', function() {
  console.log('connected to server');

  client.write('P1******(0.2,0.9,1.88,-2,139.8875,-0.0092)(123,456)(0.11,0.22,0.33,0.44,0.55,0.66)\n');
  setTimeout(function() {
    client.write('P1******(-999.9,-0.9,-1.88,2,139.8875,0.0092)(123,456)(-0.11,-0.22,-0.33,-0.44,-0.55,-0.66)\n');
  }, 1000);

//  client.destroy();
});
