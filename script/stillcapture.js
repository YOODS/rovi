#!/usr/bin/env node

'use strict';

const ros = require('rosnodejs');
const std_srvs = ros.require('std_srvs').srv;


let gRosNode = null;


async function callLowDoStillCapture(srvClPath, req, res)
{
  ros.log.info("callLowDoStillCapture() start. srvClPath=" + srvClPath);

  const srvCl = gRosNode.serviceClient(srvClPath, std_srvs.Trigger);

  await gRosNode.waitForService(srvClPath, 2000).then(async function(available)
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

  ros.log.info("callLowDoStillCapture() end.   srvClPath=" + srvClPath);

  return true;
}


async function lrLowStillCapture(req, res)
{
  ros.log.info("lrLowStillCapture() start.");

  const res_l = new std_srvs.Trigger.Response();
  const res_r = new std_srvs.Trigger.Response();

  const result = await Promise.all([
    callLowDoStillCapture('/rovi/low/cam_l/do_still_capture', req, res_l),
    callLowDoStillCapture('/rovi/low/cam_r/do_still_capture', req, res_r)
  ]);
  ros.log.info("result=" + result);

  ros.log.info("res_l.message=" + res_l.message);
  ros.log.info("res_r.message=" + res_r.message);

  if (res_l.success && res_r.success)
  {
    ros.log.info("all OK!");
    res.success = true;
    res.message = "OK: '/rovi/low/still_capture'";
  }
  else {
    ros.log.error("not all OK");
    res.success = false;
    res.message = "Failed: '/rovi/low/still_capture' ... Left[" + res_l.message + "], Right[" + res_r.message + "]";
  }

  ros.log.info("lrLowStillCapture() end.");

  return;
}


async function lowStillCapture(req, res)
{
  ros.log.info("service called: '/rovi/low/still_capture'");

  res.success = false;
  res.message = "before await lrLowStillCapture()";

  await lrLowStillCapture(req, res);

  ros.log.info("service done:   '/rovi/low/still_capture'");

  return true;
}


ros.initNode('/rovi/stillcapture').then((rosNode)=>
{
  gRosNode = rosNode;

  // Low Service still_capture
  const lowsrv_still_capture = rosNode.advertiseService('/rovi/low/still_capture', std_srvs.Trigger, lowStillCapture);
}
);

