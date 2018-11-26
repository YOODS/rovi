#!/usr/bin/env node

const execSync = require('child_process').execSync;
const yaml=require('./gvdump.js');
let contents=yaml.dump('sxga','rovi');
execSync('rosparam load -',{input:contents});

