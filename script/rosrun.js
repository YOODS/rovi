const EventEmitter=require('events').EventEmitter;
let ev=new EventEmitter();
ev.running=false;

let popen=require('child_process').exec;
popen.run=function(args,ns){
	let env=process.env;
	if(arguments.length>1) env=Object.assign(env,{ROS_NAMESPACE:ns});
	let proc=popen('rosrun '+args,{env:process.env});
	let stm=setTimeout(function(){
		ev.running=true;
		ev.emit('start'); //rosrun success
	},3000);
	proc.stdout.on('data',function(data){
		console.log('child_process:stdout:'+data);
	});
	proc.stderr.on('data',function(data){
		console.log('child_process:stderr:'+data);
	});
	proc.on('close',function(code){
		if(ev.running){
			ev.running=false;
			ev.emit('stop');
		}
		else clearTimeout(stm);
		setTimeout(function(){
			popen.start(cmd);
		},3000);
	});
	return ev;
}

module.exports=popen;
