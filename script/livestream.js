#!/usr/bin/env node

'use strict';

const ros = require('rosnodejs');
const std_srvs = ros.require('std_srvs').srv;


let gRosNode = null;


async function callLowDoLiveSet(srvClPath, toON, req, res)
{
  ros.log.info("callLowDoLiveSet() start. srvClPath=" + srvClPath + ", toON=" + toON);

  const srvCl = gRosNode.serviceClient(srvClPath, std_srvs.SetBool);

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
        ros.log.info('call ' + srvCl.getService() + ' toON=' + toON + ' returned');
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

  ros.log.info("callLowDoLiveSet() end.   srvClPath=" + srvClPath + ", toON=" + toON);

  return true;
}


async function lrLowLiveSet(toON, req, res)
{
  ros.log.info("lrLowLiveSet() start. toON=" + toON);

  const res_l = new std_srvs.SetBool.Response();
  const res_r = new std_srvs.SetBool.Response();

  const result = await Promise.all([
    callLowDoLiveSet('/rovi/left/do_live_set', toON, req, res_l),
    callLowDoLiveSet('/rovi/right/do_live_set', toON, req, res_r)
  ]);
  ros.log.info("result=" + result);

  ros.log.info("res_l.message=" + res_l.message);
  ros.log.info("res_r.message=" + res_r.message);

  if (res_l.success && res_r.success)
  {
    ros.log.info("all OK!");
    res.success = true;
    res.message = "OK: '/rovi/do_live_set " + toON + "'";
  }
  else {
    ros.log.error("not all OK");
    res.success = false;
    res.message = "Failed: '/rovi/do_live_set " + toON + "' ... Left[" + res_l.message + "], Right[" + res_r.message + "]";
  }

  ros.log.info("lrLowLiveSet() end.   toON=" + toON);

  return;
}


async function lowLiveSet(req, res)
{
  const toON = req.data;

  ros.log.info("service called: '/rovi/do_live_set' toON=" + toON);

  res.success = false;
  res.message = "before await lrLowLiveSet(" + toON+ ")";

  await lrLowLiveSet(toON, req, res);

  ros.log.info("service done:   '/rovi/do_live_set' toON=" + toON);

  return true;
}


ros.initNode('/rovi/livestream').then((rosNode)=>
{
  gRosNode = rosNode;

  // Low Service
  rosNode.advertiseService('/rovi/do_live_set', std_srvs.SetBool, lowLiveSet);
}
);

