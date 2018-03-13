#!/usr/bin/env node

'use strict';

const ros = require('rosnodejs');
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const dyn_srvs = ros.require('dynamic_reconfigure').srv;
const dyn_msgs = ros.require('dynamic_reconfigure').msg;
const rovi_srvs = ros.require('rovi').srv;


let gRosNode = null;


function usage()
{
  console.log("Usage: rosrun rovi main_node.js [--mode run|test|rc]");
}


async function callLowLiveSet(toON, req, res)
{
  ros.log.info("callLowLiveSet() start. toON=" + toON);

  let srvCl = gRosNode.serviceClient('/rovi/low/live_set', std_srvs.SetBool);

  await gRosNode.waitForService(srvCl.getService(), 500).then(async function(available)
  {
    if (!available)
    {
      let err_msg = 'service NOT available: ' + srvCl.getService();
      ros.log.error(err_msg);
      if (res)
      {
        res.success = false;
        res.message = err_msg; 
      }
      return true;
    }
    else
    {
      ros.log.info('waitForService ' + srvCl.getService() + ' OK');
      let clreq = new std_srvs.SetBool.Request();
      clreq.data = toON;
      await srvCl.call(clreq).then(function(clresp)
      {
        let info_msg = 'call ' + srvCl.getService() + ' toON=' + toON + ' returned';
        ros.log.info(info_msg);
        if (res)
        {
          res.success = clresp.success;
          res.message = clresp.message; 
        }
        return true;
      }
      );
    }
  }
  );

  ros.log.info("callLowLiveSet() end.   toON=" + toON);

  return true;
}


async function upperLiveStart(req, res)
{
  ros.log.info("service called: '/rovi/upper/live_start'");

  res.success = false;
  res.message = "before await callLowLiveSet(true)";

  await callLowLiveSet(true, req, res);

  if (res.success)
  {
    res.message = "OK: '/rovi/upper/live_start'";
  }

  ros.log.info("service done:   '/rovi/upper/live_start'");

  return true;
}


async function upperLiveStop(req, res)
{
  ros.log.info("service called: '/rovi/upper/live_stop'");

  res.success = false;
  res.message = "before await callLowLiveSet(false)";

  await callLowLiveSet(false, req, res);

  if (res.success)
  {
    res.message = "OK: '/rovi/upper/live_stop'";
  }

  ros.log.info("service done:   '/rovi/upper/live_stop'");

  return true;
}


function runMode()
{
  ros.log.info("runMode() start.");

  ros.initNode('/rovi/main_node').then((rosNode)=>
  {

    gRosNode = rosNode;

    // Run Mode starts with Live ON
    callLowLiveSet(true);

    // TODO /rovi/upper/phaseshift_capture_and_genpc
    // TODO /rovi/middle/phaseshift_capture
    // TODO /rovi/middle/phaseshift_genpc
    // TODO /rovi/upper/status_get
    // TODO /rovi/upper/param_get
    // TODO /rovi/upper/param_set
    // TODO /rovi/upper/recipe_save
    // TODO /rovi/upper/recipe_load

  }
  );

  ros.log.info("runMode() end.");
}
   

function testMode()
{
  ros.log.info("testMode() start.");

  ros.initNode('/rovi/main_node').then((rosNode)=>
  {

    gRosNode = rosNode;

    // Test Mode starts with Live OFF
    callLowLiveSet(false);

    // Upper Service live_start
    const upsrv_live_start = rosNode.advertiseService('/rovi/upper/live_start', std_srvs.Trigger, upperLiveStart);

    // Upper Service live_stop
    const upsrv_live_stop = rosNode.advertiseService('/rovi/upper/live_stop', std_srvs.Trigger, upperLiveStop);

    // Upper Service still_capture
    const upsrv_still_capture = rosNode.advertiseService( '/rovi/upper/still_capture', std_srvs.Trigger, (req,res) =>
    {
      ros.log.info("service called: '/rovi/upper/still_capture'");
      res.success = true;
      res.message = 'TODO: still_capture';
      return true;
    }
    );

    // TODO /rovi/upper/phaseshift_capture_and_genpc
    // TODO /rovi/middle/phaseshift_capture
    // TODO /rovi/middle/phaseshift_genpc
    // TODO /rovi/upper/status_get
    // TODO /rovi/upper/param_get
    // TODO /rovi/upper/param_set
    // TODO /rovi/upper/recipe_save
    // TODO /rovi/upper/recipe_load
    // TODO /rovi/upper/calib_capture
    // TODO /rovi/upper/calib_calc

  }
  );

  ros.log.info("testMode() end.");
}




// *** main start ***

const args = require('minimist')(process.argv.slice(2));
//console.log(args);  


let mode = args['mode'];
if (mode == null)
{
  mode = 'run';
}
ros.log.info('mode=[' + mode + ']');  


if (mode == 'run')
{
  runMode();
}
else if (mode == 'test')
{
  testMode();
}
else if (mode == 'rc')
{
  //
  ros.log.info("TODO rc mode");
}
else
{
  usage();
}

