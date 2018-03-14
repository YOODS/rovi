#!/usr/bin/env node

'use strict';

const ros = require('rosnodejs');
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;


let gRosNode = null;


async function callLowDoLiveSet(srvClPath, toON, req, res)
{
  ros.log.info("callLowDoLiveSet() start. srvClPath=" + srvClPath + ", toON=" + toON);

  let srvCl = gRosNode.serviceClient(srvClPath, std_srvs.SetBool);

  await gRosNode.waitForService(srvClPath, 500).then(async function(available)
  {
    if (!available)
    {
      let err_msg = 'service NOT available: ' + srvCl.getService();
      ros.log.error(err_msg);
      res.success = false;
      res.message = err_msg; 
      return true;
    }
    else
    {
      ros.log.info('waitForService ' + srvCl.getService() + ' OK');
      await srvCl.call(req).then(function(clresp)
      {
        let info_msg = 'call ' + srvCl.getService() + ' toON=' + toON + ' returned';
        ros.log.info(info_msg);
        res.success = clresp.success;
        res.message = clresp.message; 
        return true;
      }
      ).catch(function(error)
      {
        let err_msg = "service call ERROR: '" + srvCl.getService() + " " + toON + "'";
        ros.log.error(err_msg);
        res.success = false;
        res.message = err_msg; 
        return true;
      }
      );
    }
  }
  );

  ros.log.info("callLowDoLiveSet() end.   srvClPath=" + srvClPath + ", toON=" + toON);

  return (res.success == true);
}


async function lrLowLiveSet(toON, req, res)
{
  ros.log.info("lrLowLiveSet() start. toON=" + toON);

  let res_l = new std_srvs.SetBool.Response();
  let res_r = new std_srvs.SetBool.Response();

  const result = await Promise.all([
    callLowDoLiveSet('/rovi/low/cam_l/do_live_set', toON, req, res_l),
    callLowDoLiveSet('/rovi/low/cam_r/do_live_set', toON, req, res_r)
  ]);
  ros.log.info("result=" + result);

  ros.log.info("res_l.message=" + res_l.message);
  ros.log.info("res_r.message=" + res_r.message);

  if (res_l.success && res_r.success)
  {
    ros.log.info("all OK!");
    res.success = true;
    res.message = "OK: '/rovi/low/live_set " + toON + "'";
  }
  else {
    ros.log.error("not all OK");
    res.success = false;
    res.message = "Failed: '/rovi/low/live_set " + toON + "' ... Left[" + res_l.message + "], Right[" + res_r.message + "]";
  }

  ros.log.info("lrLowLiveSet() end.   toON=" + toON);

  return;
}


async function lowLiveSet(req, res)
{
  let toON = req.data;

  ros.log.info("service called: '/rovi/low/live_set' toON=" + toON);

  res.success = false;
  res.message = "before await callLowDoLiveSet(" + toON+ ")";

  await lrLowLiveSet(toON, req, res);

  ros.log.info("service done:   '/rovi/low/live_set' toON=" + toON);

  return true;
}


ros.initNode('/rovi/livestream').then((rosNode)=>
{
  gRosNode = rosNode;

  // Low Service live_set
  const lowsrv_live_set = rosNode.advertiseService('/rovi/low/live_set', std_srvs.SetBool, lowLiveSet);
}
);

