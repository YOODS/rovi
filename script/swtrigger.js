#!/usr/bin/env node

'use strict';

const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const dyn_msgs = ros.require('dynamic_reconfigure').msg;
const dyn_srvs = ros.require('dynamic_reconfigure').srv;


let gRosNode = null;

let myNodeName;
let path_SrvSr_DoLiveSet;
let path_TpcSub_ImageRaw;
let path_SrvCl_Queue;
let path_SrvCl_SetParam;


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


async function lowDoLiveSet(req, res)
{
  ros.log.info("lowDoLiveSet start.");

  const toON = req.data;

  ros.log.info("service called: '" + path_SrvSr_DoLiveSet + "' toON=" + toON);

  res.success = false;
  res.message = "before low live set toON=" + toON;

  // stop live anyway
//  clearTimeout(camTriggerWDT);
  gRosNode.unsubscribe(path_TpcSub_ImageRaw);
  camTriggerClient = null;

  const srvCl_setparam = gRosNode.serviceClient(path_SrvCl_SetParam, dyn_srvs.Reconfigure, {persist:true});

  await gRosNode.waitForService(srvCl_setparam.getService(), 500).then(async function(available)
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

      let request = new dyn_srvs.Reconfigure.Request();
      let param1 = new dyn_msgs.StrParameter();
      param1.name = 'TriggerMode';
      param1.value = 'On';
      request.config.strs.push(param1);
      let param2 = new dyn_msgs.StrParameter();
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

      await srvCl_setparam.call(request).then(async function(clresp)
      {
        const info_msg = 'call ' + srvCl_setparam.getService() + ' toON=' + toON + ' returned';
        ros.log.info(info_msg);

        if (!toON)
        {
          res.success = true;
          res.message = "OK: '" + path_SrvSr_DoLiveSet + " " + toON + "'";

          ros.log.info("service done:   '" + path_SrvSr_DoLiveSet + "' toON=" + toON);

          return true;
        }
        // toON call camera/queue
        else
        {
          ros.log.info("call camera/queue");

          const tpcSub_imageraw = gRosNode.subscribe(path_TpcSub_ImageRaw, sensor_msgs.Image, (img_raw) =>
          {
            //camOK();
            ros.log.debug('subscribed: ' + path_TpcSub_ImageRaw);
            ros.log.debug("call camera/queue after subscribe"); 
            camTrigger();
          }
          );

          const srvCl_queue = gRosNode.serviceClient(path_SrvCl_Queue, std_srvs.Trigger, {persist:true});

          await gRosNode.waitForService(srvCl_queue.getService(), 500).then(async function(available)
          {
            if (!available)
            {
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
        let err_msg = "service call ERROR: '" + srvCl_setparam.getService() + "'";
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


function init()
{
  ros.initNode(myNodeName).then((rosNode)=>
  {
    gRosNode = rosNode;

    // Low Service do_live_set
    const lowsrv_doliveset = rosNode.advertiseService(path_SrvSr_DoLiveSet, std_srvs.SetBool, lowDoLiveSet);
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
  path_TpcSub_ImageRaw = '/rovi/cam_l/camera/image_raw';
  path_SrvCl_Queue = '/rovi/cam_l/camera/queue';
  path_SrvCl_SetParam = '/rovi/cam_l/camera/set_parameters';
  init();
}
else if (lr == 'r')
{
  myNodeName = '/rovi/cam_r/swtrigger';
  path_SrvSr_DoLiveSet = '/rovi/low/cam_r/do_live_set';
  path_TpcSub_ImageRaw = '/rovi/cam_r/camera/image_raw';
  path_SrvCl_Queue = '/rovi/cam_r/camera/queue';
  path_SrvCl_SetParam = '/rovi/cam_r/camera/set_parameters';
  init();
}
else
{
  usage();
}

