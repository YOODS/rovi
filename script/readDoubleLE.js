#!/usr/bin/env node

const fs = require('fs');

if (process.argv.length < 3) {
  console.log('Missing file name');
  return;
}

let buf = fs.readFileSync(process.argv[2]);
let fbuf = [];

for (let i = 0; i < buf.length; i += 8) {
  let val = buf.readDoubleLE(i);
  fbuf.push(val);
  if (fbuf.length >= 4) {
    console.log(fbuf.toString());
    fbuf = [];
  }
}
