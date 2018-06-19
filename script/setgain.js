#!/usr/bin/env node

'use strict';

const ws=require('./wsshare.js').listen(5000);
const ros = require('rosnodejs');
const dyn_srvs = ros.require('dynamic_reconfigure').srv;
const dyn_msgs = ros.require('dynamic_reconfigure').msg;

var target=null;

ros.initNode('/rovi/setgain').then((rosNode)=>{
	var cli=rosNode.serviceClient('camera/set_parameters',dyn_srvs.Reconfigure,{persist:true});
	rosNode.waitForService(cli.getService(),2000).then(function(available){
		if(!available){
			ros.log.info('Service not available');
			return;
		}
		target=cli;
	});
});

ws.Gain=function(x){
	if(target==null) return;
	var request=new dyn_srvs.Reconfigure.Request();
	var param=new dyn_msgs.DoubleParameter();
	param.name='Gain';
	param.value=x;
	request.config.doubles.push(param);
	target.call(request).then(function(resp){
		ros.log.info('Gain called');
	});	
}
