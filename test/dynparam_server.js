#!/usr/bin/env node

'use strict';
const ros = require('rosnodejs');
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const dyn_srvs = ros.require('dynamic_reconfigure').srv;
const dyn_msgs = ros.require('dynamic_reconfigure').msg;
const rovi_srvs = ros.require('rovi').srv;

var url,port;

ros.initNode('/rovi/projector').then((rosNode)=>{
	const svc_dyn=rosNode.advertiseService('/rovi/projector/set_parameters',dyn_srvs.Reconfigure, (req,res)=>{
		req.config.ints.forEach(function(elm){
			ros.log.info('projector::IntParam '+elm.name+'='+elm.value);
		});
		req.config.strs.forEach(function(elm){
			ros.log.info('projector::StrParam '+elm.name+'='+elm.value);
		});
		req.config.doubles.forEach(function(elm){
			ros.log.info('projector::DoubleParam '+elm.name+'='+elm.value);
		});
		return true;
	});
});
