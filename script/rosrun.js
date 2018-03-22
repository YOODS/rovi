const EventEmitter=require('events').EventEmitter;
let ev=new EventEmitter();
ev.running=false;

let popen=require('child_process');
popen.run=function(node,ns){
	let env=process.env;
	let args=arguments;
	if(args.length>1) env=Object.assign(env,{ROS_NAMESPACE:ns});
	let proc=popen.exec('rosrun '+node,{env:process.env});
//	let proc=popen.spawn('rosrun',node.slice(' '),{env:process.env});
	let stm=setTimeout(function(){
		ev.running=true;
		ev.emit('start'); //rosrun success
	},3000);
	proc.stdout.on('data',function(data){
		console.log('rosrun:stdout:'+data);
		ev.emit('cout',data);
	});
	proc.stderr.on('data',function(data){
		console.log('rosrun:stderr:'+data);
		ev.emit('cerr',data);
	});
	proc.on('close',function(code){
		console.log('rosrun closed:'+code);
		if(ev.running){
			ev.running=false;
			ev.emit('stop');
		}
		else clearTimeout(stm);
		setTimeout(function(){
			popen.run.apply(popen,args);
		},3000);
	});
	return ev;
}

module.exports=popen;
