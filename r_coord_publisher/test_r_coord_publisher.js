#!/usr/bin/env node

const net = require('net');

var client = new net.Socket();

client.connect(10002, '127.0.0.1', function() {
  console.log('connected to server');

  client.write('P1******(0.2,0.9,1.88,-2,139.8875,-0.0092)(123,456)(-28.43,33.11,85.43,20.44,109.17,-47.78)\n');
  setTimeout(function() {
    client.write('P1******(-999.9,-0.9,-1.88,2,139.8875,0.0092)(123,456)(-34.56,-179.87,-0.33,123.45,0,90.0)\n');
  }, 1000);

//  client.destroy();
});
