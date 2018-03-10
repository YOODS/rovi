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
	const svc0=rosNode.advertiseService('/rovi/projector/set_parameters',dyn_srvs.Reconfigure, (req,res)=>{
		req.config.ints.forEach(function(elm){
			ros.log.info('projector::IntParam '+elm.name+'='+elm.value);
		});
		req.config.strs.forEach(function(elm){
			ros.log.info('projector::StrParam '+elm.name+'='+elm.value);
		});
		return true;
	});
	const svc1=rosNode.advertiseService('/rovi/projector/enable',std_srvs.SetBool, (req,res)=>{
		ros.log.info('projector::enable');
		return res.success=true;
	});
	const svc2=rosNode.advertiseService('/rovi/projector/query',std_srvs.Trigger, (req,res)=>{
		ros.log.info('projector::query');
		res.message='YPJ4500 OK';
		return res.success=true;
	});
	const svc3=rosNode.advertiseService('/rovi/projector/save',rovi_srvs.DigitalFilter, (req,resp)=>{
		return true;
	});
	const svc4=rosNode.advertiseService('/rovi/projector/load',rovi_srvs.DigitalFilter, (req,resp)=>{
		return true;
	});
	const pub=rosNode.advertise('/rovi/projector/ready',std_msgs.Bool);
	rosNode.getParam('projector/address').then(function(value){
		url=value;
		ros.log.info('projector::url='+url);
	});
});
