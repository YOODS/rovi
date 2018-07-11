const Net = require('net');
const EventEmitter = require('events').EventEmitter;
const Runner = require('./runner.js');
const Notifier = new EventEmitter();

const ros = require('rosnodejs');
const sensor_msgs = ros.require('sensor_msgs').msg;
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const rovi_srvs = ros.require('rovi').srv;

const shm = require('../shm-typed-array');

let shmem;

let run_c;  // should be rosrun.js camera runner
let run_p;  // should be openYPJ:Socket

let imgLength;

let lrSeq = 0;
let lrStamp = 0;

function imgCreate(param) {
  let img = new sensor_msgs.Image();
  img.header.seq = 0;
  img.header.stamp = 0;
  img.header.frame_id = 'camera';
  img.width = 1280;
  img.height = 1024;
  img.step = 1280;
  img.encoding = 'mono8';
  for (let key in img) {
    if (param.hasOwnProperty(key)) {
      img[key] = param[key];
    }
  }
  imgLength = img.height * img.step;
  return img;
}

function copyImg(src) {
  let img = new sensor_msgs.Image();
  img.header.seq = src.header.seq;
  img.header.stamp = src.header.stamp;
  img.header.frame_id = src.header.frame_id;
  img.width = src.width;
  img.height = src.height;
  img.step = src.step;
  img.encoding = src.encoding;
  img.data = src.data.slice(0, img.width * img.height);
  return img;
}

let image_l;
let image_r;

function msleep(t) {
  return new Promise(function(resolve) {
    setTimeout(function() {
      resolve(true);
    }, t);
  });
}

var ycam = {
  isliveon: async function() {
    // TODO
    let ison = true;
    return ison;
  },
  cset: function(obj) {
    for (let key in obj) {
      console.log("ycam1s cset key=" + key + ", val=" + obj[key]);
      run_c.cin(key + ' ' + obj[key]);
    }
    return 'OK';
  },
  pset: function(obj) {
    for (let key in obj) {
      console.log("ycam1s pset key=" + key + ", val=" + obj[key]);
      switch(key){
      case 'ExposureTime':
        run_p.write('x' + obj[key]+'\n');
        break;
      case 'Interval':
        run_p.write('p' + obj[key]+'\n');
        break;
      case 'Intencity':
        let ix = obj[key] < 256 ? obj[key] : 255;
        ix=ix.toString(16);
        run_p.write('i'+ix+ix+ix+'\n');
        break;
      case 'Go':
        let gx= obj[key]<2? obj[key]:2;
        run_p.write('p'+gx+'\n');
        break;
      }
      run_p.setNoDelay(true);
    }
    return 'OK';
  },
  normal: false,
  stat: function() {
    return { 'camera': run_c.running, 'projector': !run_p.destroyed };
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
  open: function(idl, idr, url, port) {
//    run_c = Runner.run('grabber-sentech ' + idl + ' ' + idr);
//    run_c = Runner.run('../basler_example/grabber');
    run_c = Runner.run(process.env.ROVI_PATH + "/sentech_grabber/grabber '" + idl + "' '" + idr + "'");
    this.scan();
    run_c.on('start', function(data) {
      console.log("ycam1s::run_c start");
      if(!run_p.destroyed) Notifier.emit('wake');
    });
    run_c.on('cout', function(data) {
      let lines = data.split(/\n/);
      for (let i = 0; i < lines.length; i++) {
        if (!lines[i]) {
          continue;
        }
        let attr;
        try {
          attr = JSON.parse(lines[i]);
//          ros.log.warn('ycam1s done parse [' + lines[i] + ']');
        }
        catch(err) {
          ros.log.error('ycam1s failed parse [' + lines[i] + ']');
          ros.log.error('ycam1s parse err[' + err +']');
          return;
        }
        if (attr.hasOwnProperty('capt')) {
          let offset = attr.capt;
          if (offset == 0) {
            image_l.header.seq++;
//            image_l.header.stamp = ros.Time.now();
            // precedent
            if (image_l.header.seq > lrSeq) {
              image_l.header.stamp = ros.Time.now();
//              ros.log.warn("lseq(" + image_l.header.seq + ") > lrSeq(" + lrSeq + "). now lstamp=" + JSON.stringify(image_l.header.stamp));
              lrSeq = image_l.header.seq;
              lrStamp = image_l.header.stamp;
//              ros.log.warn("now lrSeq=" + lrSeq + ", lrStamp=" + JSON.stringify(lrStamp));
            }
            // catch up
            else if (image_l.header.seq == lrSeq) {
              image_l.header.stamp = lrStamp;
//              ros.log.warn("lseq(" + image_l.header.seq + ") == lrSeq(" + lrSeq + "). now lstamp=" + JSON.stringify(image_l.header.stamp));
            }
            // far behind
            else {
              image_l.header.seq = lrSeq;
              image_l.header.stamp = lrStamp;
//              ros.log.warn("lseq(" + image_l.header.seq + ") < lrSeq(" + lrSeq + "). now lseq=" + image_l.header.seq + ", lstamp=" + JSON.stringify(image_l.header.stamp));
            }
            image_l.data = shmem.slice(0, imgLength);
            Notifier.emit('left', copyImg(image_l));
          }
          else {
            image_r.header.seq++;
//            image_r.header.stamp = ros.Time.now();
            // precedent
            if (image_r.header.seq > lrSeq) {
              image_r.header.stamp = ros.Time.now();
//              ros.log.warn("rseq(" + image_r.header.seq + ") > lrSeq(" + lrSeq + "). now rstamp=" + JSON.stringify(image_r.header.stamp));
              lrSeq = image_r.header.seq;
              lrStamp = image_r.header.stamp;
//              ros.log.warn("now lrSeq=" + lrSeq + ", lrStamp=" + JSON.stringify(lrStamp));
            }
            // catch up
            else if (image_r.header.seq == lrSeq) {
              image_r.header.stamp = lrStamp;
//              ros.log.warn("rseq(" + image_r.header.seq + ") == lrSeq(" + lrSeq + "). now rstamp=" + JSON.stringify(image_r.header.stamp));
            }
            // far behind
            else {
              image_r.header.seq = lrSeq;
              image_r.header.stamp = lrStamp;
//              ros.log.warn("rseq(" + image_r.header.seq + ") < lrSeq(" + lrSeq + "). now rseq=" + image_r.header.seq + ", rstamp=" + JSON.stringify(image_r.header.stamp));
            }
            image_r.data = shmem.slice(offset, offset + imgLength);
            Notifier.emit('right', copyImg(image_r));
          }
        }
        else if (attr.hasOwnProperty('shm')) {
          shmem = shm.get(attr.shm, 'Uint8Array');
          delete attr.shm;
          image_l = imgCreate(attr);
          image_r = imgCreate(attr);
console.log('shm size:' + shmem.byteLength);
        }
      }
    });
    run_p = openYPJ(port, url);
    run_p.on('data', function(data) {
      if (data == 'Dlp.X>') {
      }
    });
    return Notifier;
  }
}

function openYPJ(port, url, sock) {
  if (arguments.length < 3) sock = new Net.Socket();
  sock.on('connect', function() {
    ros.log.info('YPJ connected');
    console.log("ycam1s::run_p ready");
    if(run_c.running) Notifier.emit('wake');
  });
  sock.on('error', function() {
    ros.log.error('YPJ error');
  });
  sock.on('close', function() {
    ros.log.info('YPJ closed');
    setTimeout(function() {
      sock.connect(port, url);
    }, 3000);
  });
  sock.connect(port, url);
  return sock;
}

module.exports = ycam;
