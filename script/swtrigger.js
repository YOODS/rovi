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


var camTriggerClient = null;
var camTriggerRequest = null;
var camTriggerWDT = null;


function camTrigger()
{
  if (camTriggerClient == null) {
    ros.log.error("lll");
    return;
  }

  camTriggerClient.call(camTriggerRequest).then(function(resp){});
  camTriggerWDT = setTimeout(
    function()
    {
      camTrigger();
    }, 100
  );
}


function camOK()
{
  clearTimeout(camTriggerWDT);
}


async function lowDoLiveSet(req, res)
{
  ros.log.info("lowDoLiveSet start.");

  let toON = req.data;

  ros.log.info("service called: '" + path_SrvSr_DoLiveSet + "' toON=" + toON);

  res.success = false;
  res.message = "before low live set toON=" + toON;

  let srvCl_setparam = gRosNode.serviceClient(path_SrvCl_SetParam, dyn_srvs.Reconfigure, {persist:true});

  await gRosNode.waitForService(srvCl_setparam.getService(), 500).then(async function(available)
  {
    if (!available)
    {
      let err_msg = 'service NOT available: ' + srvCl_setparam.getService();
      ros.log.error(err_msg);

      res.success = false;
      res.message = err_msg; 

      return false;
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

      await srvCl_setparam.call(request).then(function(clresp)
      {
        let info_msg = 'call ' + srvCl_setparam.getService() + ' toON=' + toON + ' returned';
        ros.log.info(info_msg);

        res.success = clresp.success;
        res.message = info_msg; 

        if (toON) {
          // TODO
          ros.log.error("TODO queue!");
        }

        return true;
      }
      );
    }
  }
  );

  res.success = true;
  res.message = 'low do_live_set to ' + toON + ' OK';

  ros.log.info("service done:   '" + path_SrvSr_DoLiveSet + "' toON=" + toON);

  ros.log.info("lowDoLiveSet end.");

  return (res.success == true);
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

/*
    let sub =
      rosNode.subscribe(
        topicImageRaw,
        sensor_msgs.Image,
        (img) =>
        {
          camOK();
          ros.log.info('subscribed: ' + topicImageRaw);
          camTrigger();
        }
      );

    var cl =
      rosNode.serviceClient(
        srvCl_Queue,
        std_srvs.Trigger,
        {persist:true}
      );


    rosNode.waitForService(cl.getService(), 2000).then(function(available)
    {
      if (!available)
      {
        ros.log.warn('Service not available: ' + srvCl_Queue);
        return;
      }

      camTriggerClient = cl;
      camTriggerRequest = new std_srvs.Trigger.Request();
      camTrigger();
    }
    );

  }
  );

}

*/



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

