#!/usr/bin/env node


const NSycamctrl = '/rovi/ycam_ctrl';
const NSpsgenpc = '/rovi/pshift_genpc';
const NScamL = '/rovi/left';
const NScamR = '/rovi/right';
const NSlive = '/rovi/live';
const NSrovi = '/rovi';
const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const sensName = process.argv.length < 3 ? 'ycam1s' : process.argv[2];
const sens = require('./' + sensName + '.js')
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const rovi_srvs = ros.require('rovi').srv;
const EventEmitter = require('events').EventEmitter;
const jsyaml = require("js-yaml");

const dbg = false;

// TODO from param_V live: camera: AcquisitionFrameRate: ?
const waitmsec_for_livestop = 200;

ros.Time.diff = function(t0) {
  let t1 = ros.Time.now();
  t1.secs -= t0.secs;
  t1.nsecs -= t0.nsecs;
  return ros.Time.toSeconds(t1);
}

class ImageSwitcher {
  constructor(node, ns) {
    const who = this;
    this.node = node;
    this.raw = node.advertise(ns + '/image_raw', sensor_msgs.Image);
    this.rect = node.advertise(ns + '/image_rect', sensor_msgs.Image);
    this.vue = node.advertise(ns + '/view', sensor_msgs.Image);
    this.info = node.advertise(ns + '/camera_info', sensor_msgs.CameraInfo);
    this.remap = node.serviceClient(ns + '/remap', rovi_srvs.ImageFilter, { persist: true });
    this.setremapparam = node.serviceClient(ns + '/remap/set_remap_param', rovi_srvs.SetRemapParam);
    setImmediate(async function() {
      if (!await node.waitForService(who.remap.getService(), 2000)) {
        ros.log.error('remap service not available');
        return;
      }
      if (!await node.waitForService(who.setremapparam.getService(), 2000)) {
        ros.log.error('set_remap_param service not available');
        return;
      }
      try {
        if (sensName === 'ycam1s') {
          who.param = await node.getParam(ns + '/camera');
          who.caminfo = Object.assign(new sensor_msgs.CameraInfo(), await node.getParam(ns + '/remap'));
        }
      }
      catch(err) {
        ros.log.warn('getParam ' + err);
      }
    });
    this.hook = new EventEmitter();
    this.capt = [];
    if (ns === NScamL) {
      this.lr = 'L';
    }
    else {
      this.lr = 'R';
    }
  }
  async emit(img) {
//    const who = this;
//    ros.log.warn('got emit img ' + who.lr + ' seq=' + img.header.seq);
    this.raw.publish(img);
    let req = new rovi_srvs.ImageFilter.Request();
    req.img = img;
//    ros.log.warn('before call remap ' + who.lr + ' seq=' + img.header.seq);
    let res = await this.remap.call(req);
//    ros.log.warn('after  call remap ' + who.lr + ' seq=' + img.header.seq);
    if (this.hook.listenerCount('store') > 0) this.hook.emit('store', res.img);
    else this.rect.publish(res.img);
    this.caminfo.header = req.img.header;
    this.caminfo.distortion_model = "plumb_bob";
    this.info.publish(this.caminfo);
//    ros.log.warn('after publish caminfo ' + who.lr + ' seq=' + img.header.seq);
  }
  store(count) {
    const who = this;
    this.capt = [];
    return new Promise(function(resolve) {
      who.hook.on('store', function(img) {
        if (dbg) {
          ros.log.warn('capturing img_' + who.lr + '['+ who.capt.length + "] seq=" + img.header.seq);
        }
        if (who.capt.length < count) {
          who.capt.push(img);
          if (dbg) {
            ros.log.warn('captured  img_' + who.lr + '['+ (who.capt.length - 1) + "] seq=" + img.header.seq + ' ... now capt.length=' + who.capt.length);
          }
          if (who.capt.length == count) {
            if (dbg) {
              ros.log.warn('now ' + count + ' img_' + who.lr + 's. resolve.');
            }
            who.hook.removeAllListeners();
            resolve(who.capt);
          }
        }
        else if (who.capt.length >= count) {
          ros.log.warn('already ' + count + ' img_' + who.lr + 's. ignore this img.');
        }
      });
    });
  }
  cancel() {
//    ros.log.warn("before this.hook.removeAllListeners()");
    this.hook.removeAllListeners();
//    ros.log.warn("after  this.hook.removeAllListeners()");
  }
  get ID() {return this.param.ID;}
  view(n) {
    if (n < this.capt.length) {
      this.vue.publish(this.capt[n]);
    }
  }
  async setCamParam(remapyaml) {
    this.caminfo = Object.assign(new sensor_msgs.CameraInfo(), remapyaml);
    let req = new rovi_srvs.SetRemapParam.Request();
    req.height = remapyaml.height;
    req.width = remapyaml.width;
    req.D = remapyaml.D;
    req.K = remapyaml.K;
    req.R = remapyaml.R;
    req.P = remapyaml.P;
    if (dbg) {
      ros.log.warn('setCamPareq height=' + req.height + ' P.length=' + req.P.length);
      ros.log.warn('before call remap set param ' + this.lr);
    }
    let res = await this.setremapparam.call(req);
    if (dbg) {
      ros.log.warn('after  call remap set param ' + this.lr);
    }
  }
}

setImmediate(async function() {
  const rosNode = await ros.initNode(NSycamctrl);
  let cam_resolution;

  try {
    let camera_size = await rosNode.getParam(NSrovi + '/camera');
//    ros.log.warn('camera_size h=' + camera_size.Height + ' w=' + camera_size.Width);

    if (camera_size.Height == 480 && camera_size.Width == 1280) {
      ros.log.warn('camera size = VGA');
      cam_resolution = 'vga';
    }
    else if (camera_size.Height == 1024 && camera_size.Width == 2560) {
      ros.log.warn('camera size = SXGA');
      cam_resolution = 'sxga';
    }
    else {
      ros.log.error('Invalid camera size');
      return;
    }
  }
  catch(err) {
    ;
  }

  const image_L = new ImageSwitcher(rosNode, NScamL);
  const image_R = new ImageSwitcher(rosNode, NScamR);
  const pub_stat = rosNode.advertise(NSycamctrl + '/stat', std_msgs.Bool);
  let vue_N = 0;
  const pub_pc = rosNode.advertise(NSrovi + '/pc', sensor_msgs.PointCloud);
  const pub_pc2 = rosNode.advertise(NSrovi + '/pc2', sensor_msgs.PointCloud2);
  const genpc = rosNode.serviceClient(NSrovi + '/genpc', rovi_srvs.GenPC, { persist: true });
  const setgenpcparam = rosNode.serviceClient(NSrovi + '/genpc/set_genpc_param', rovi_srvs.SetGenpcParam);
  if (!await rosNode.waitForService(genpc.getService(), 2000)) {
    ros.log.error('genpc service not available');
    return;
  }
  if (!await rosNode.waitForService(setgenpcparam.getService(), 2000)) {
    ros.log.error('set_genpc_param service not available');
    return;
  }

  let param_C = await rosNode.getParam(NSpsgenpc + '/camera');
  let param_V = await rosNode.getParam(NSlive + '/camera');
  let param_P = await rosNode.getParam(NSpsgenpc + '/projector');
  let paramTimer = null;
  function paramDiff(o, n) {
    let c = Object.assign({}, n);
    for (let key in c) {
      if (o.hasOwnProperty(key)) {
        if (o[key] == c[key]) delete c[key];
      }
    }
    return c;
  }
  async function paramReloadNow() {
    if (dbg) {
      ros.log.warn('>>>>*********paramReloadNow called');
    }
    param_C = await rosNode.getParam(NSpsgenpc + '/camera');
    let nv = await rosNode.getParam(NSlive + '/camera');
    let np = await rosNode.getParam(NSpsgenpc + '/projector');
    try {
      await sens.cset(paramDiff(param_V, nv));
      param_V = nv;
      await sens.pset(paramDiff(param_P, np));
      param_P = np;
    }
    catch(err) {
      ros.log.warn('Exception in paramReloadNow:' + err);
      return;
    }
    if (dbg) {
      ros.log.warn('<<<<*********paramReloadNow end');
    }
  }
  async function paramReload() {
    if (dbg) {
      ros.log.warn('>>***********paramReload called');
    }
    param_C = await rosNode.getParam(NSpsgenpc + '/camera');
    let nv = await rosNode.getParam(NSlive + '/camera');
    let np = await rosNode.getParam(NSpsgenpc + '/projector');
    if (paramTimer != null) {
      try {
        await sens.cset(paramDiff(param_V, nv));
        param_V = nv;
        await sens.pset(paramDiff(param_P, np));
        param_P = np;
      }
      catch(err) {
        ros.log.warn('Exception in paramReload:' + err);
        paramTimer = null;
        return;
      }
    }
    if (paramTimer != null) paramTimer = setTimeout(paramReload, 1000);
    if (dbg) {
      ros.log.warn('<<***********paramReload end');
    }
  }
  function paramScan() {
    if (dbg) {
      ros.log.warn('paramScan CALLED');
    }
    if (paramTimer == null) paramTimer = setTimeout(paramReload,1000);
  }
  function paramStop() {
    if (paramTimer != null) {
      clearTimeout(paramTimer);
    }
    paramTimer = null;
  }

  let sensEv;
  switch (sensName) {
  case 'ycam1s':
    sensEv = sens.open(image_L.ID, image_R.ID, param_P.Url, param_P.Port);
    break;
  case 'ycam3':
    sensEv = sens.open(rosNode, NSrovi, cam_resolution);
    break;
  }
  sensEv.on('stat', function(s) {
    let f = new std_msgs.Bool();
    f.data = s;
    pub_stat.publish(f);
  });
  sensEv.on('wake', async function(yamlstr) {
    if (dbg) {
      ros.log.warn('wake. yamlstr=[' + yamlstr + ']');
    }
    const yamlval = jsyaml.safeLoad(yamlstr);
//    ros.log.warn('lh=' + yamlval.left.remap.height);
//    ros.log.warn('rw=' + yamlval.right.remap.width);
//    ros.log.warn('Q=' + yamlval.genpc.Q);
    await image_L.setCamParam(yamlval.left.remap);
    await image_R.setCamParam(yamlval.right.remap);
    let req = new rovi_srvs.SetGenpcParam.Request();
    req.Q = yamlval.genpc.Q;
    if (dbg) {
      ros.log.warn('before call genpc set param');
    }
    let res = await setgenpcparam.call(req);
    if (dbg) {
      ros.log.warn('after  call genpc set param');
    }
    param_V = {};
    param_P = {};
    paramStop();
    await paramReloadNow();
    await sens.cset({ 'TriggerMode': 'On' }); // live OFF anyway
    paramScan();
    ros.log.warn('NOW ALL READY');
  });
  sensEv.on('left', async function(img) {
    if (dbg) {
      ros.log.warn("from ycam left seq=" + img.header.seq);
    }
    image_L.emit(img);
  });
  sensEv.on('right', async function(img) {
    if (dbg) {
      ros.log.warn("from ycam right seq=" + img.header.seq);
    }
    image_R.emit(img);
  });

// ---------Definition of services
  let capt_L; // <--------captured images of the left camera
  let capt_R; // <--------same as right
  const svc_do = rosNode.advertiseService(NSpsgenpc, std_srvs.Trigger, (req, res) => { // <--------generate PCL
if (dbg) {
ros.log.warn('service pshift_genpc called');
}
    paramStop();
    if (!sens.normal) {
      ros.log.warn(res.message = 'YCAM not ready');
      res.success = false;
      paramScan();
      return true;
    }
    return new Promise(async (resolve) => {
//      const timeoutmsec = param_P.Interval * 20;
      // TODO tmp +10 and +280
      const timeoutmsec = (param_P.Interval + 10) * 20 + waitmsec_for_livestop + 280 + 7000;
if (dbg) {
ros.log.warn('livestop and pshift_genpc setTimeout ' + timeoutmsec + ' msec');
}
      let wdt = setTimeout(async function() { // <--------watch dog
        resolve(false);
        image_L.cancel();
        image_R.cancel();
//        await sens.cset({ 'TriggerMode': 'Off' });
        paramScan();
        ros.log.error('livestop and pshift_genpc timed out');
      }, timeoutmsec);
      param_V = Object.assign(param_V, param_C);
//      await sens.cset({ 'TriggerMode': 'On' });
      await sens.cset(param_C);
if (dbg) {
ros.log.warn('now await livestop and pshift_genpc');
}
await setTimeout(async function() {
if (dbg) {
ros.log.warn("after livestop, pshift_genpc function start");
}
      await sens.pset({'Go':2}); // <--------projector sequence start
if (dbg) {
ros.log.warn('after pset p2');
}

      let imgs = await Promise.all([image_L.store(13), image_R.store(13)]);
if (dbg) {
ros.log.warn('after await Promise.all (image_L.store(13) and image_R.store(13) resolve)');
}
      image_L.cancel();
      image_R.cancel();
      clearTimeout(wdt);
      capt_L = imgs[0];
      capt_R = imgs[1];
if (dbg) {
ros.log.warn('capt_L and capt_R set. capt_L.length=' + capt_L.length + ", capt_R.length=" + capt_R.length);
}

if (dbg) {
      for (let li = 0; li < capt_L.length; li++) {
        ros.log.warn("Set capt_L[" + li + "].seq=" + capt_L[li].header.seq);
      }
      for (let ri = 0; ri < capt_R.length; ri++) {
        ros.log.warn("Set capt_R[" + ri + "].seq=" + capt_R[ri].header.seq);
      }

      ros.log.warn("genpc CALL");
}
      let gpreq = new rovi_srvs.GenPC.Request();
      gpreq.imgL = capt_L;
      gpreq.imgR = capt_R;
      let gpres = await genpc.call(gpreq);
      pub_pc.publish(gpres.pc);
      pub_pc2.publish(gpres.pc2);
if (dbg) {
      ros.log.warn('pc published');
      ros.log.warn("genpc DONE");
}
//      await sens.cset({ 'TriggerMode': 'Off' });
      await sens.cset({ 'TriggerMode': 'On' }); // live OFF anyway
      paramScan();
      res.message = imgs[0].length + ' images scan compelete. Generated PointCloud Count=' + gpres.pc.points.length;
      res.success = true;
      image_L.view(vue_N);
      image_R.view(vue_N);
      resolve(true);

if (dbg) {
ros.log.warn("pshift_genpc function end");
ros.log.warn('service pshift_genpc resolve true return');
}
      }, waitmsec_for_livestop); // これはライブの残りカスを捨てるための待ち

    });
  });

  const svc_parse = rosNode.advertiseService(NSycamctrl + '/parse', rovi_srvs.Dialog, async (req, res) => {
    let cmd = req.hello;
    let lbk = cmd.indexOf('{');
    let obj = {};
    if (lbk > 0) {
      cmd = req.hello.substring(0, lbk).trim();
      try {
        obj = JSON.parse(req.hello.substring(lbk));
      }
      catch(err) {
        // ignore
      }
    }
    let cmds = cmd.split(' ');
    if (cmds.length > 1) cmd = cmds.shift();
    switch (cmd) {
    case 'cset':
      if (!sens.normal) {
        res.answer = 'YCAM not ready';
      }
      else {
        res.answer = 'OK';
        await sens.cset(obj);
      }
      return Promise.resolve(true);
    case 'pset':
      if (!sens.normal) {
        res.answer = 'YCAM not ready';
      }
      else {
        res.answer = 'OK';
        await sens.pset(obj);
      }
      return Promise.resolve(true);
    case 'stat': // <--------sensor (maybe YCAM) status query
      return new Promise((resolve) => {
        res.answer = JSON.stringify(sens.stat());
        resolve(true);
      });
    case 'view':
      return new Promise((resolve) => {
        try {
          vue_N = parseInt(cmds[0]);
        }
        catch(err) {
          vue_N = 0;
        }
        image_L.view(vue_N);
        image_R.view(vue_N);
        resolve(true);
      });
    }
  });
});
