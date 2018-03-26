#!/usr/bin/env node

'use strict';

const ros = require('rosnodejs');
const std_srvs = ros.require('std_srvs').srv;
const rovi_srvs = ros.require('rovi').srv;


let gRosNode = null;


function usage()
{
  console.log("Usage: rosrun rovi main_node.js [--mode run|test|rc]");
}


async function callLowLiveSet(toON, req, res)
{
  ros.log.info("callLowLiveSet() start. toON=" + toON);

  const srvCl = gRosNode.serviceClient('/rovi/low/live_set', std_srvs.SetBool);

  await gRosNode.waitForService(srvCl.getService(), 2000).then(async function(available)
  {
    if (!available)
    {
      const err_msg = 'service NOT available: ' + srvCl.getService();
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
      const clreq = new std_srvs.SetBool.Request();
      clreq.data = toON;
      await srvCl.call(clreq).then(function(clres)
      {
        ros.log.info('call ' + srvCl.getService() + ' toON=' + toON + ' returned');
        if (res)
        {
          res.success = clres.success;
          res.message = clres.message;
        }
        return true;
      }
      ).catch(function(error)
      {
        const err_msg = "service call ERROR: '" + srvCl.getService() + " " + toON + "' (" + error + ")";
        ros.log.error(err_msg);
        res.success = false;
        res.message = err_msg; 
        return true;
      }
      );
    }
  }
  );

  ros.log.info("callLowLiveSet() end.   toON=" + toON);

  return true;
}


async function callLowStillCapture(req, res)
{
  ros.log.info("callLowStillCapture() start.");

  const srvCl = gRosNode.serviceClient('/rovi/low/still_capture', std_srvs.Trigger);

  await gRosNode.waitForService(srvCl.getService(), 2000).then(async function(available)
  {
    if (!available)
    {
      const err_msg = 'service NOT available: ' + srvCl.getService();
      ros.log.error(err_msg);
      res.success = false;
      res.message = err_msg; 
      return true;
    }
    else
    {
      ros.log.info('waitForService ' + srvCl.getService() + ' OK');
      const clreq = new std_srvs.Trigger.Request();
      await srvCl.call(clreq).then(function(clres)
      {
        ros.log.info('call ' + srvCl.getService() + ' returned');
        res.success = clres.success;
        res.message = clres.message;
        return true;
      }
      ).catch(function(error)
      {
        const err_msg = "service call ERROR: '" + srvCl.getService() + "' (" + error + ")";
        ros.log.error(err_msg);
        res.success = false;
        res.message = err_msg; 
        return true;
      }
      );
    }
  }
  );

  ros.log.info("callLowStillCapture() end.");

  return true;
}


async function callLowParamGet(req, res)
{
  ros.log.info("callLowParamGet() start.");

  const srvCl = gRosNode.serviceClient('/rovi/low/param_get', rovi_srvs.GetParam);

  await gRosNode.waitForService(srvCl.getService(), 2000).then(async function(available)
  {
    if (!available)
    {
      const err_msg = 'service NOT available: ' + srvCl.getService();
      ros.log.error(err_msg);
      res.success = false;
      res.message = err_msg; 
      return true;
    }
    else
    {
      ros.log.info('waitForService ' + srvCl.getService() + ' OK');
      await srvCl.call(req).then(function(clres)
      {
        ros.log.info('call ' + srvCl.getService() + ' returned');
        res.success = clres.success;
        res.message = clres.message;
        res.value = clres.value;
        return true;
      }
      ).catch(function(error)
      {
        const err_msg = "service call ERROR: '" + srvCl.getService() + "' (" + error + ")";
        ros.log.error(err_msg);
        res.success = false;
        res.message = err_msg; 
        return true;
      }
      );
    }
  }
  );

  ros.log.info("callLowParamGet() end.");

  return true;
}


async function callLowParamSet(req, res)
{
  ros.log.info("callLowParamSet() start.");

  const srvCl = gRosNode.serviceClient('/rovi/low/param_set', rovi_srvs.SetParam);

  await gRosNode.waitForService(srvCl.getService(), 2000).then(async function(available)
  {
    if (!available)
    {
      const err_msg = 'service NOT available: ' + srvCl.getService();
      ros.log.error(err_msg);
      res.success = false;
      res.message = err_msg; 
      return true;
    }
    else
    {
      ros.log.info('waitForService ' + srvCl.getService() + ' OK');
      await srvCl.call(req).then(function(clres)
      {
        ros.log.info('call ' + srvCl.getService() + ' returned');
        res.success = clres.success;
        res.message = clres.message;
        return true;
      }
      ).catch(function(error)
      {
        const err_msg = "service call ERROR: '" + srvCl.getService() + "' (" + error + ")";
        ros.log.error(err_msg);
        res.success = false;
        res.message = err_msg; 
        return true;
      }
      );
    }
  }
  );

  ros.log.info("callLowParamSet() end.");

  return true;
}


async function upperLiveStart(req, res)
{
  ros.log.info("service called: '/rovi/live_start'");

  res.success = false;
  res.message = "before await callLowLiveSet(true)";

  await callLowLiveSet(true, req, res);

  if (res.success)
  {
    res.message = "OK: '/rovi/live_start'";
  }

  ros.log.info("service done:   '/rovi/live_start'");

  return true;
}


async function upperLiveStop(req, res)
{
  ros.log.info("service called: '/rovi/live_stop'");

  res.success = false;
  res.message = "before await callLowLiveSet(false)";

  await callLowLiveSet(false, req, res);

  if (res.success)
  {
    res.message = "OK: '/rovi/live_stop'";
  }

  ros.log.info("service done:   '/rovi/live_stop'");

  return true;
}


async function upperStillCapture(req, res)
{
  ros.log.info("service called: '/rovi/still_capture'");

  res.success = false;
  res.message = "before await callLowStillCapture()";

  await callLowStillCapture(req, res);

  if (res.success)
  {
    res.message = "OK: '/rovi/still_capture'";
  }

  ros.log.info("service done:   '/rovi/still_capture'");

  return true;
}


async function upperParamGet(req, res)
{
  ros.log.info("service called: '/rovi/param_get'");

  res.success = false;
  res.message = "before await callLowParamGet()";
  res.value = "";

  await callLowParamGet(req, res);

  if (res.success)
  {
    res.message = "OK: '/rovi/param_get'";
  }

  ros.log.info("service done:   '/rovi/param_get'");

  return true;
}


async function upperParamSet(req, res)
{
  ros.log.info("service called: '/rovi/param_set'");

  res.success = false;
  res.message = "before await callLowParamSet()";

  await callLowParamSet(req, res);

  if (res.success)
  {
    res.message = "OK: '/rovi/param_set'";
  }

  ros.log.info("service done:   '/rovi/param_set'");

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

    // Upper Services
    rosNode.advertiseService('/rovi/param_get', rovi_srvs.GetParam, upperParamGet);
    rosNode.advertiseService('/rovi/param_set', rovi_srvs.SetParam, upperParamSet);

    // TODO /rovi/pshift_capture_and_genpc
    // TODO /rovi/pshift_capture
    // TODO /rovi/pshift_genpc
    // TODO /rovi/status_get
    // TODO /rovi/recipe_save
    // TODO /rovi/recipe_load

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

    // Upper Services
    rosNode.advertiseService('/rovi/live_start', std_srvs.Trigger, upperLiveStart);
    rosNode.advertiseService('/rovi/live_stop', std_srvs.Trigger, upperLiveStop);
    rosNode.advertiseService( '/rovi/still_capture', std_srvs.Trigger, upperStillCapture); 
    rosNode.advertiseService('/rovi/param_get', rovi_srvs.GetParam, upperParamGet);
    rosNode.advertiseService('/rovi/param_set', rovi_srvs.SetParam, upperParamSet);

    // TODO /rovi/pshift_capture_and_genpc
    // TODO /rovi/pshift_capture
    // TODO /rovi/pshift_genpc
    // TODO /rovi/status_get
    // TODO /rovi/recipe_save
    // TODO /rovi/recipe_load
    // TODO /rovi/calib_capture
    // TODO /rovi/calib_calc

    // TODO
    rosNode.setParam('/rovi/testparam', {'p': 1, 'i': 'str', 'd': 3});
    rosNode.setParam('/rovi/testparam2', [0.0, 1.1, 2.2]);
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

