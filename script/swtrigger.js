#!/usr/bin/env node

'use strict';

const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const dyn_msgs = ros.require('dynamic_reconfigure').msg;
const dyn_srvs = ros.require('dynamic_reconfigure').srv;
const rovi_srvs = ros.require('rovi').srv;


let gRosNode = null;

let myNodeName;

let path_SrvSr_DoLiveSet;
let path_SrvSr_DoStillCapture;
let path_TpcSub_ImageRaw;
let path_TpcPub_Live_ImageRaw;
let path_TpcPub_Live_ImageRect;
let path_TpcPub_Still_ImageRaw;
let path_TpcPub_Still_ImageRect;
let path_SrvCl_SetParam;
let path_SrvCl_Queue;
let path_SrvCl_RemapDo;

let tpcPub_live_imageraw;
let tpcPub_live_imagerect;
let tpcPub_still_imageraw;
let tpcPub_still_imagerect;


function usage()
{
  console.log("Usage: rosrun rovi swtrigger.js [--lr l|r]");
}


let camTriggerClient = null;
//let camTriggerRequest = null;
let camTriggerRequest = new std_srvs.Trigger.Request();
//let camTriggerWDT = null;


function camTrigger()
{
  ros.log.debug("called camTrigger()");

  if (camTriggerClient == null)
  {
    ros.log.warn("camTriggerClient is null");
    return;
  }

  camTriggerClient.call(camTriggerRequest).then(function(resp){});
  ros.log.debug("service call " + camTriggerClient.getService() + " done");

//  camTriggerWDT = setTimeout(
//    function()
//    {
//      ros.log.info("call camTrigger in timer");
//      camTrigger();
//    }, 100
//  );

}


//function camOK()
//{
//  clearTimeout(camTriggerWDT);
//}


function stopTriggerAll()
{
//  clearTimeout(camTriggerWDT);
  gRosNode.unsubscribe(path_TpcSub_ImageRaw);
  camTriggerClient = null;
}


async function callLowRemapDoAndPubRawRect(img_raw, tpcPub_imageraw, tpcPub_imagerect)
{
  ros.log.debug("callLowRemapDoAndPubRawRect() start.");

  const srvCl = gRosNode.serviceClient(path_SrvCl_RemapDo, rovi_srvs.ImageFilter);

  await gRosNode.waitForService(srvCl.getService(), 2000).then(async function(available)
  {
    if (!available)
    {
      ros.log.error('service NOT available: ' + srvCl.getService());
      return false;
    }
    else
    {
      ros.log.debug('waitForService ' + srvCl.getService() + ' OK');
      const clreq = new rovi_srvs.ImageFilter.Request();
      clreq.img = img_raw;
      await srvCl.call(clreq).then(function(clres)
      {
        ros.log.debug('call ' + srvCl.getService() + ' returned');

        tpcPub_imageraw.publish(img_raw);
        tpcPub_imagerect.publish(clres.img);
        ros.log.debug('published: ' + tpcPub_imageraw.getTopic() + ' and ' + tpcPub_imagerect.getTopic());

        return true;
      }
      ).catch(function(error)
      {
        ros.log.error("service call ERROR: '" + srvCl.getService() + "' (" + error + ")");
        return false;
      }
      );
    }
  }
  );

  ros.log.debug("callLowRemapDoAndPubRawRect() end.");

  return true;
}


async function lowDoLiveSet(req, res)
{
  ros.log.info("lowDoLiveSet start.");

  const toON = req.data;

  ros.log.info("service called: '" + path_SrvSr_DoLiveSet + "' toON=" + toON);

  res.success = false;
  res.message = "before low live set toON=" + toON;

  // stop live anyway
  stopTriggerAll();

  const srvCl_setparam = gRosNode.serviceClient(path_SrvCl_SetParam, dyn_srvs.Reconfigure, {persist:true});

  await gRosNode.waitForService(srvCl_setparam.getService(), 2000).then(async function(available)
  {
    if (!available)
    {
      const err_msg = 'service NOT available: ' + srvCl_setparam.getService();
      ros.log.error(err_msg);

      res.success = false;
      res.message = err_msg; 

      return true;
    }
    else
    {
      ros.log.info('waitForService ' + srvCl_setparam.getService() + ' OK');

      const request = new dyn_srvs.Reconfigure.Request();
      const param1 = new dyn_msgs.StrParameter();
      param1.name = 'TriggerMode';
      param1.value = 'On';
      request.config.strs.push(param1);
      const param2 = new dyn_msgs.StrParameter();
      param2.name = 'TriggerSource';
      if (toON)
      {
        param2.value = 'Software';
        //param2.value = 'SW--livestart';
      }
      else
      {
        param2.value = 'Line1';
        //param2.value = 'HW--livestop';
      }
      request.config.strs.push(param2);

      await srvCl_setparam.call(request).then(async function(clres)
      {
        ros.log.info('call ' + srvCl_setparam.getService() + ' toON=' + toON + ' returned');

        if (!toON)
        {
          res.success = true;
          res.message = "OK: '" + path_SrvSr_DoLiveSet + " " + toON + "'";

          ros.log.info("service done:   '" + path_SrvSr_DoLiveSet + "' toON=" + toON);

          return true;
        }
        // toON ... call camera/queue
        else
        {
          ros.log.info("call camera/queue");

          const tpcSub_imageraw = gRosNode.subscribe(path_TpcSub_ImageRaw, sensor_msgs.Image, async (img_raw) =>
          {
            //camOK();

            ros.log.debug('subscribed: ' + path_TpcSub_ImageRaw);

            await callLowRemapDoAndPubRawRect(img_raw, tpcPub_live_imageraw, tpcPub_live_imagerect);

            ros.log.debug("call camera/queue after callLowRemapDoAndPubRawRect()"); 
            camTrigger();
          }
          );

          const srvCl_queue = gRosNode.serviceClient(path_SrvCl_Queue, std_srvs.Trigger, {persist:true});

          await gRosNode.waitForService(srvCl_queue.getService(), 2000).then(async function(available)
          {
            if (!available)
            {
              stopTriggerAll();

              const err_msg = 'service NOT available: ' + srvCl_queue.getService();
              ros.log.error(err_msg);

              res.success = false;
              res.message = err_msg; 

              return true;
            }
            else
            {
              camTriggerClient = srvCl_queue;
              //camTriggerRequest = new std_srvs.Trigger.Request();
              ros.log.info("call camera/queue first"); 
              camTrigger();

              res.success = true;
              res.message = "OK: '" + path_SrvSr_DoLiveSet + " " + toON + "'";

              ros.log.info("service done:   '" + path_SrvSr_DoLiveSet + "' toON=" + toON);

              return true;
            }
          }
          );
        }
      }
      ).catch(function(error)
      {
        const err_msg = "service call ERROR: '" + srvCl_setparam.getService() + "' (" + error + ")";
        ros.log.error(err_msg);
        res.success = false;
        res.message = err_msg; 
        return true;
      }
      );
    }
  }
  );

  ros.log.info("lowDoLiveSet end.");

  return true;
}


async function lowDoStillCapture(req, res)
{
  ros.log.info("lowDoStillCapture start.");

  ros.log.info("service called: '" + path_SrvSr_DoStillCapture + "'");

  res.success = false;
  res.message = "before low still capture";

  // stop live anyway
  stopTriggerAll();

  const srvCl_setparam = gRosNode.serviceClient(path_SrvCl_SetParam, dyn_srvs.Reconfigure, {persist:true});

  await gRosNode.waitForService(srvCl_setparam.getService(), 2000).then(async function(available)
  {
    if (!available)
    {
      const err_msg = 'service NOT available: ' + srvCl_setparam.getService();
      ros.log.error(err_msg);

      res.success = false;
      res.message = err_msg; 

      return true;
    }
    else
    {
      ros.log.info('waitForService ' + srvCl_setparam.getService() + ' OK');

      const request = new dyn_srvs.Reconfigure.Request();
      const param1 = new dyn_msgs.StrParameter();
      param1.name = 'TriggerMode';
      param1.value = 'On';
      request.config.strs.push(param1);
      const param2 = new dyn_msgs.StrParameter();
      param2.name = 'TriggerSource';
      param2.value = 'Software';
      request.config.strs.push(param2);

      await srvCl_setparam.call(request).then(async function(clres)
      {
        ros.log.info('call ' + srvCl_setparam.getService() + ' returned');

        // still capture ... call camera/queue once
        {
          ros.log.info("call camera/queue for still");

          const tpcSub_imageraw = gRosNode.subscribe(path_TpcSub_ImageRaw, sensor_msgs.Image, async (img_raw) =>
          {
            //camOK();

            ros.log.debug('subscribed: ' + path_TpcSub_ImageRaw);

            await callLowRemapDoAndPubRawRect(img_raw, tpcPub_still_imageraw, tpcPub_still_imagerect);

            // Not call camTrigger() ... 'cause we call camera/queue only once for still capture

            stopTriggerAll();
          }
          );

          const srvCl_queue = gRosNode.serviceClient(path_SrvCl_Queue, std_srvs.Trigger, {persist:true});

          await gRosNode.waitForService(srvCl_queue.getService(), 2000).then(async function(available)
          {
            if (!available)
            {
              stopTriggerAll();

              const err_msg = 'service NOT available: ' + srvCl_queue.getService();
              ros.log.error(err_msg);

              res.success = false;
              res.message = err_msg; 

              return true;
            }
            else
            {
              camTriggerClient = srvCl_queue;
              //camTriggerRequest = new std_srvs.Trigger.Request();
              ros.log.info("call camera/queue once"); 
              camTrigger();

              res.success = true;
              res.message = "OK: '" + path_SrvSr_DoStillCapture + "'";

              ros.log.info("service done:   '" + path_SrvSr_DoStillCapture + "'");

              return true;
            }
          }
          );
        }
      }
      ).catch(function(error)
      {
        const err_msg = "service call ERROR: '" + srvCl_setparam.getService() + "' (" + error + ")";
        ros.log.error(err_msg);
        res.success = false;
        res.message = err_msg; 
        return true;
      }
      );
    }
  }
  );

  ros.log.info("lowDoStillCapture end.");

  return true;
}


function init()
{
  ros.initNode(myNodeName).then((rosNode)=>
  {
    gRosNode = rosNode;

    tpcPub_live_imageraw = rosNode.advertise(path_TpcPub_Live_ImageRaw, sensor_msgs.Image);
    tpcPub_live_imagerect = rosNode.advertise(path_TpcPub_Live_ImageRect, sensor_msgs.Image);

    tpcPub_still_imageraw = rosNode.advertise(path_TpcPub_Still_ImageRaw, sensor_msgs.Image);
    tpcPub_still_imagerect = rosNode.advertise(path_TpcPub_Still_ImageRect, sensor_msgs.Image);

    // Low Service do_live_set
    const lowsrv_doliveset = rosNode.advertiseService(path_SrvSr_DoLiveSet, std_srvs.SetBool, lowDoLiveSet);

    // Low Service do_still_capture
    const lowsrv_dostillcapture = rosNode.advertiseService(path_SrvSr_DoStillCapture, std_srvs.Trigger, lowDoStillCapture);
  }
  );
}




// *** start ***

const args = require('minimist')(process.argv.slice(2));

let lr = args['lr'];
if (lr == null)
{
  lr = 'l';
}
ros.log.info('lr=[' + lr + ']');  

if (lr == 'l')
{
  myNodeName = '/rovi/cam_l/swtrigger';
  path_SrvSr_DoLiveSet = '/rovi/low/cam_l/do_live_set';
  path_SrvSr_DoStillCapture = '/rovi/low/cam_l/do_still_capture';
  path_TpcSub_ImageRaw = '/rovi/cam_l/camera/image_raw';
  path_TpcPub_Live_ImageRaw = '/rovi/cam_l/live/image_raw';
  path_TpcPub_Live_ImageRect = '/rovi/cam_l/live/image_rect';
  path_TpcPub_Still_ImageRaw = '/rovi/cam_l/still/image_raw';
  path_TpcPub_Still_ImageRect = '/rovi/cam_l/still/image_rect';
  path_SrvCl_SetParam = '/rovi/cam_l/camera/set_parameters';
  path_SrvCl_Queue = '/rovi/cam_l/camera/queue';
  path_SrvCl_RemapDo = '/rovi/cam_l/remap/do';
  init();
}
else if (lr == 'r')
{
  myNodeName = '/rovi/cam_r/swtrigger';
  path_SrvSr_DoLiveSet = '/rovi/low/cam_r/do_live_set';
  path_SrvSr_DoStillCapture = '/rovi/low/cam_r/do_still_capture';
  path_TpcSub_ImageRaw = '/rovi/cam_r/camera/image_raw';
  path_TpcPub_Live_ImageRaw = '/rovi/cam_r/live/image_raw';
  path_TpcPub_Live_ImageRect = '/rovi/cam_r/live/image_rect';
  path_TpcPub_Still_ImageRaw = '/rovi/cam_r/still/image_raw';
  path_TpcPub_Still_ImageRect = '/rovi/cam_r/still/image_rect';
  path_SrvCl_SetParam = '/rovi/cam_r/camera/set_parameters';
  path_SrvCl_Queue = '/rovi/cam_r/camera/queue';
  path_SrvCl_RemapDo = '/rovi/cam_r/remap/do';
  init();
}
else
{
  usage();
}

