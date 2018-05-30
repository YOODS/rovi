#!/usr/bin/env node
//
//Test program of performance of getParam
//

const ros=require('rosnodejs');

let rosNode;
let params;
let count=0;
let t1,t2;
function scan_param(name){
	let promis=rosNode.getParam('param');
	promis.then(function ok(p){
		t2=ros.Time.now();
		params=p;
		count++;
		if(count<10) scan_param(name);
		else{
			console.log(p);
			console.log(ros.Time.toSeconds(t2)-ros.Time.toSeconds(t1));
		}
	});
}

setImmediate(async function(){
	rosNode=await ros.initNode('param_perf');
	t1=ros.Time.now();
	scan_param('param/P1');
});
