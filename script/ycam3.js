const Net = require('net');
const EventEmitter = require('events').EventEmitter;
const Rosrun = require('./rosrun.js');
const Notifier = new EventEmitter;

const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const dyn_msgs = ros.require('dynamic_reconfigure').msg;
const dyn_srvs = ros.require('dynamic_reconfigure').srv;
const gev_srvs = ros.require('camera_aravis').srv;
const rovi_srvs = ros.require('rovi').srv;

const execSync = require('child_process').execSync;

const dbg = false;

let run_c;  // should be rosrun.js camnode
let rosNode;

const reg_table = {
  'AcquisitionStart': 0x00100018,
  'AcquisitionStop': 0x0010001c,
  'AcquisitionFrameRate': 0x00101000,
  'ExposureTime': 0x00101004,
  'GainAnalog': 0x00101008,
  'Gain': 0x0010100C,
  'TriggerMode': 0x00101010,
  'SerialPort': 0x00300004
}

const val_table = {
  'Off': 0,
  'On': 1
}

var ycam = {
  isliveon: async function() {
    let ison = false;
    let greq = new gev_srvs.GevRegs.Request();
    greq.address = reg_table['TriggerMode'];
    try {
      gres = await run_c.reg_read.call(greq);
      if (gres.data === 0) {
        ison = true;
      }
    }
    catch(err) {
      let warnmsg = 'YCAM3 isliveon TriggerMode read ' + err;
      ros.log.warn(warnmsg);
    }
    return ison;
  },
  cset: async function(obj) {
    let ret = 'OK';
    let greq = new gev_srvs.GevRegs.Request();
    let dreq = new dyn_srvs.Reconfigure.Request();
    for (let key in obj) {
      let val = obj[key];
      if (reg_table.hasOwnProperty(key)) {
        if (dbg) {
          ros.log.warn('key=' + key + ', val=' + val + '. reg_write...');
        }
        greq.address = reg_table[key];
        greq.data = typeof(val) == 'string' ? val_table[val] : val;
        try {
          await run_c.reg_write.call(greq);
        }
        catch(err) {
          ros.log.error('YCAM3 cset write ' + err);
          ret = 'YCAM not ready';
        }
        if (dbg) {
          ros.log.warn('key=' + key + ', val=' + val + '. reg_write... done.');
        }
      }
      else if (typeof(val) == 'string') {
        if (dbg) {
          ros.log.warn('key=' + key + ', val=' + val + '. dynparam str');
        }
        let param = new dyn_msgs.StrParameter();
        param.name = key;
        param.value = val;
        dreq.config.strs.push(param);
      }
      else {
        if (dbg) {
          ros.log.warn('key=' + key + ', val=' + val + '. dynparam double');
        }
        let param = new dyn_msgs.DoubleParameter();
        param.name = key;
        param.value = val;
        dreq.config.doubles.push(param);
      }
    }
    if (dreq.config.strs.length > 0 || dreq.config.doubles.length > 0) {
      await run_c.dynparam_set.call(dreq);
      if (dbg) {
        ros.log.warn('dynparam_set done.');
      }
    }
    return ret;
  },
  pregbuf: '',
  pregwrt: async function() {
    if (dbg) {
      ros.log.warn('pregwrt called. pregbuf.length=' + ycam.pregbuf.length);
    }
    let success = true;
    if (ycam.pregbuf.length > 0) {
      let greq = new gev_srvs.GevRegs.Request();
      greq.address = reg_table['SerialPort'];
      let lsb = ycam.pregbuf.charCodeAt(0);
      if (dbg) {
        ros.log.warn('lsb=' + lsb);
      }
      greq.data = (~lsb << 16) | lsb;
      await run_c.reg_write.call(greq);

      ycam.pregbuf = ycam.pregbuf.slice(1);
      await ycam.pregwrt();
    }
  },
  pset: async function(obj) {
    if (dbg) {
      ros.log.warn('pset called');
    }
    let ret = 'OK';
    let str = '';
    for (let key in obj) {
      if (dbg) {
        ros.log.warn('pset key=' + key + ', val=' + obj[key]);
      }
      switch (key) {
      case 'ExposureTime':
        str += '\n' + 'x' + obj[key] + '\n';
        break;
      case 'Interval':
        str += '\n' + 'o' + obj[key] + '\n';
        break;
      case 'Intencity':
        let ix = obj[key] < 256 ? obj[key] : 255;
        ix = ix.toString(16);
        str += '\n' + 'i' + ix + ix + ix + '\n';
        break;
      case 'Go':
        let gx = obj[key] < 2 ? obj[key] : 2;
        str += '\n' + 'o' + gx + '\n';
        break;
      }
    }
    this.pregbuf += str;
    if (dbg) {
      ros.log.warn('l=' + this.pregbuf.length);
    }
    try {
      await this.pregwrt();
    }
    catch(err) {
      ros.log.error('YCAM3 pset write ' + err);
      ret = 'YCAM not ready';
      this.pregbuf = '';
    }

    if (dbg) {
      ros.log.warn('pset DONE');
    }

    return ret;
  },
  normal: false,
  stat: function() {
    return { 'cam': run_c.running };
  },
  scan: function() {
    let s;
    try {
      s = this.stat();
    }
    catch(err) {
      Notifier.emit('stat', this.normal = false);
      setTimeout(function() {ycam.scan();}, 1000);
      return;
    }
    let f = true;
    for (let key in s) f = f && s[key];
    if (f == undefined) f = false;
    Notifier.emit('stat', this.normal = f);
    setTimeout(function() {ycam.scan();}, 1000);
  },
  open: function(nh, ns, resolution) {
    ros.log.warn('YCAM3 Opening...');
    rosNode = nh;
    run_c = Rosrun.run('camera_aravis camnode', ns);
    run_c.on('start', async function() {

//      ros.log.warn('before execSync get_genicam_xml');
      const xmlstr = execSync('arv-tool-0.4 genicam | tail -n +2 | tee /tmp/genicam.xml').toString();
//      ros.log.warn('after  execSync get_genicam_xml ... xmlstr=[' + xmlstr + ']');
//      ros.log.warn('after  execSync get_genicam_xml');

      let lines = xmlstr.split(/\n/);

      const yoods = '==YOODS==';

      let vga_yamlstr ='';
      let sxga_yamlstr ='';

      let curkey = '';

      for (let li = 0; li < lines.length; li++) {
//        ros.log.warn('lines[' + li + ']=[' + lines[li] + ']');

        if (lines[li].startsWith(yoods)) {
          const key = lines[li].slice(yoods.length);
//          ros.log.warn('key=[' + key + ']');
          if (key === 'start') {
            // ignore
          }
          else if (key === 'vga') {
            curkey = 'vga';
          }
          else if (key === 'sxga') {
            curkey= 'sxga';
          }
          else if (key === 'end') {
            break;
          }
          else {
            ros.log.error(yoods + ' UNKNOWN key...[' + key + ']');
            process.exit(102);
          }
        }
        else {
          if (curkey === 'vga') {
            vga_yamlstr += lines[li] + '\n';
          }
          else if (curkey === 'sxga') {
            sxga_yamlstr += lines[li] + '\n';
          }
        }
      }

//      ros.log.warn('vga_yamlstr=[' + vga_yamlstr + ']');
//      ros.log.warn('sxga_yamlstr=[' + sxga_yamlstr + ']');

      let yamlstr = '';
      if (resolution === 'vga') {
         yamlstr = vga_yamlstr;
      }
      else if (resolution === 'sxga') {
         yamlstr = sxga_yamlstr;
      }

      if (yamlstr.length === 0) {
        ros.log.error('Cannot get Camera Parameter');
        process.exit(103);
      }

      if (!await openCamera(run_c, ns)) {
        ros.log.error('Failure in openCamera');
        process.exit(101);
      }
      Notifier.emit('wake', yamlstr);
    });

    run_c.on('stop', async function() {
      ros.log.warn('YCAM3 Stopped');
      rosNode.unsubscribe(ns + '/camera/image_raw');
    });

    this.scan();
    return Notifier;
  }
}

async function openCamera(rosrun, ns) {
  let sub = rosNode.subscribe(ns + '/camera/image_raw', sensor_msgs.Image, (src) => {
//    ros.log.warn('ycam3.js:: Got seq=' + src.header.seq);
    let image_l = new sensor_msgs.Image();
    let image_r = new sensor_msgs.Image();
    image_l.data = new Uint8Array();
    image_r.data = new Uint8Array();
    const h = image_l.height = image_r.height = src.height;
    const w = image_l.step = image_r.step = src.step / 2;
    image_l.header = image_r.header = src.header;
    image_l.width = image_r.width = src.width / 2;
    image_l.encoding = image_r.encoding = src.encoding;
    let sz = image_l.step * image_l.height;
    if (sz > image_l.data.byteLength) image_l.data = new Uint8Array(sz);
    if (sz > image_r.data.byteLength) image_r.data = new Uint8Array(sz);
    for (let i = 0, s = 0, t = 0; i < h; i++, t += w) {
      image_l.data.subarray(t).set(src.data.subarray(s, s + w));
      s += w;
      image_r.data.subarray(t).set(src.data.subarray(s, s + w));
      s += w;
    }
    Notifier.emit('left', image_l);
    Notifier.emit('right', image_r);
//    ros.log.warn('ycam3.js:: after emit seq=' + src.header.seq);
  }, { queueSize: 20 });
  return new Promise(async function(resolve) {
    let regw = rosNode.serviceClient(ns + '/camera/regw', gev_srvs.GevRegs, { persist: true });
    if (await rosNode.waitForService(regw.getService(), 2000)) {
      rosrun.reg_write = regw;
    }
    else {
      ros.log.error(ns + '/"regw" not available');
      resolve(false);
    }
    let regr = rosNode.serviceClient(ns + '/camera/regr', gev_srvs.GevRegs, { persist: true });
    if (await rosNode.waitForService(regr.getService(), 2000)) {
      rosrun.reg_read = regr;
    }
    else {
      ros.log.error(ns + '/"regr" not available');
      resolve(false);
    }
    let cset = rosNode.serviceClient(ns + '/camera/set_parameters', dyn_srvs.Reconfigure, { persist: true });
    if (await rosNode.waitForService(cset.getService(), 2000)) {
      rosrun.dynparam_set = cset;
    }
    else {
      ros.log.error(ns + '/"set_parameters" not available');
      resolve(false);
    }
    resolve(true);
  });
}

module.exports = ycam;
