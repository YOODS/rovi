#!/usr/bin/env node

'use strict';

const ros = require('rosnodejs');
const yaml = require("js-yaml");
const std_srvs = ros.require('std_srvs').srv;
const dyn_msgs = ros.require('dynamic_reconfigure').msg;
const dyn_srvs = ros.require('dynamic_reconfigure').srv;
const rovi_srvs = ros.require('rovi').srv;


let gRosNode = null;


async function callLowDoParamGet(req, res)
{
  const paramName = req.name;

  ros.log.info("callLowDoParamGet() start. paramName=" + paramName);

  await gRosNode.hasParam(paramName).then(async function(exists)
  {
    if (!exists)
    {
      const err_msg = "Not exists parameter '" + paramName + "'";
      ros.log.error(err_msg);
      res.success = false;
      res.message = err_msg; 
      res.value = '';
      return true;
    }

    await gRosNode.getParam(paramName).then(async function(paramValue)
    {
      const yamlValue = yaml.safeDump(paramValue);
      ros.log.info('getParam(' + paramName + ') returned. paramValue=' + yamlValue);
      res.success = true;
      res.message = "OK: '/rovi/do_param_get'";
      res.value = yamlValue;
      return true; 
    }
    ).catch(function(error)
    {
      const err_msg = 'getParam(' + paramName + ') ERROR: (' + error + ')';
      ros.log.error(err_msg);
      res.success = false;
      res.message = err_msg; 
      res.value = '';
      return true;
    }
    );
  }
  );

  ros.log.info("callLowDoParamGet() end.   paramName=" + paramName);

  return true;
}


async function callLowDoParamSet(req, res)
{
  const paramName = req.name;
  const paramVal = yaml.safeLoad(req.value);

  ros.log.info("callLowDoParamSet() start. paramName=" + paramName + ", paramVal=" + paramVal);

  await gRosNode.hasParam(paramName).then(async function(exists)
  {
    if (!exists)
    {
      const err_msg = "Not exists parameter '" + paramName + "'";
      ros.log.error(err_msg);
      res.success = false;
      res.message = err_msg; 
      return true;
    }

    let needDynSet = false;
    let ppath;
    let leaf;

    let canonParamName = paramName + '/';
    canonParamName = canonParamName.replace(/\/+/g, '/');
    canonParamName = canonParamName.slice(0, -1);
    ros.log.debug("canonParamName=" + canonParamName);
    const mres = canonParamName.match(/.*\//);
    if (mres == null)
    {
      needDynSet = false;
    }
    else
    {
      ppath = mres[0];
      leaf = canonParamName.replace(ppath, '');
      ros.log.debug("ppath=" + ppath);
      ros.log.debug("leaf=" + leaf);
  
      if (ppath == '/rovi/left/camera/image_raw/compressed/' ||
          ppath == '/rovi/left/camera/image_raw/compressedDepth/' ||
          ppath == '/rovi/left/camera/image_raw/theora/' ||
          ppath == '/rovi/left/camera/' ||
          ppath == '/rovi/right/camera/image_raw/compressed/' ||
          ppath == '/rovi/right/camera/image_raw/compressedDepth/' ||
          ppath  == '/rovi/right/camera/image_raw/theora/' ||
          ppath  == '/rovi/right/camera/')
      {
        needDynSet = true;
      }
      else
      {
        needDynSet = false;
      }
    }

    if (needDynSet)
    {
      const srvCl_dynsetparam = gRosNode.serviceClient(ppath + "set_parameters", dyn_srvs.Reconfigure, {persist:true});

      await gRosNode.waitForService(srvCl_dynsetparam.getService(), 2000).then(async function(available)
      {
        if (!available)
        {
          const err_msg = 'service NOT available: ' + srvCl_dynsetparam.getService();
          ros.log.error(err_msg);
          res.success = false;
          res.message = err_msg; 
          return true;
        }
        else
        {
          ros.log.info('waitForService ' + srvCl_dynsetparam.getService() + ' OK');

          const request = new dyn_srvs.Reconfigure.Request();
          const param = new dyn_msgs.StrParameter(); // TODO ParamDescription ?
          param.name = leaf;
          param.value = paramVal;
          request.config.strs.push(param);

          await srvCl_dynsetparam.call(request).then(async function(clres)
          {
            ros.log.info('call ' + srvCl_dynsetparam.getService() + ' returned');
            res.success = true;
            res.message = "OK: '/rovi/do_param_set'";
            return true;
          }
          ).catch(function(error)
          {
            const err_msg = "service call ERROR: '" + srvCl_dynsetparam.getService() + "' (" + error + ")";
            ros.log.error(err_msg);
            res.success = false;
            res.message = err_msg; 
            return true;
          }
          );
        }
      }
      );
    }
    else
    {
      gRosNode.setParam(paramName, paramVal);
      ros.log.info('setParam(' + paramName + ', ' + paramVal + ') returned');
      res.success = true;
      res.message = "OK: '/rovi/do_param_set'";
      return true; 
    }
  }
  );

  ros.log.info("callLowDoParamSet() end.   paramName=" + paramName + ", paramVal=" + paramVal);

  return true;
}


async function lowParamGet(req, res)
{
  ros.log.info("service called: '/rovi/do_param_get'");

  res.success = false;
  res.message = "before await callLowDoParamGet()";

  await callLowDoParamGet(req, res),

  ros.log.info("service done:   '/rovi/do_param_get'");

  return true;
}


async function lowParamSet(req, res)
{
  ros.log.info("service called: '/rovi/do_param_set'");

  res.success = false;
  res.message = "before await callLowDoParamSet()";

  await callLowDoParamSet(req, res),

  ros.log.info("service done:   '/rovi/do_param_set'");

  return true;
}


ros.initNode('/rovi/parameter').then((rosNode)=>
{
  gRosNode = rosNode;

  // Low Services
  rosNode.advertiseService('/rovi/do_param_get', rovi_srvs.GetParam, lowParamGet);
  rosNode.advertiseService('/rovi/do_param_set', rovi_srvs.SetParam, lowParamSet);
}
);

