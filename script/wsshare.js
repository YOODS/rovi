var WSH = require('ws');

WSH.listen = function(p) {
  var wss = this.ws = new this.Server({ port: p });
  wss.on('connection', function(client) {
    console.log("New connection");
    client.sendObject = function(obj) {
      var s = JSON.stringify(arguments.length > 0 ? obj : wss);
      client.send(s);
    }
    client.on('message', function(msg) {
      console.log('received:' + msg);
      var obj = JSON.parse(msg);
      for (var key in obj) {
        var val = obj[key];
        if ((typeof wss[key]) === 'undefined') {
          console.log('WS:set undefined, so make it:' + key);
          wss[key] = val;
        }
        else if ((typeof wss[key]) === 'function') {
          console.log('WS:set.call:' + key + '=' + val);
          wss[key](val, client);
        }
        else {
          console.log('WS:set.val:' + key + '=' + val);
          wss[key] = val;
        }
      }
    });
    client.on("close", function() {
      console.log("Connection closed");
    });
  });
  return wss;
}

WSH.parse = function(rc) {
  eval(rc);
}

module.exports = WSH;
