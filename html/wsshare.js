var WsShare=function(url,func){
	this.sock=new WebSocket(url);
	var evptr=this;
	var evcb=arguments.length>=2? func:null;
//	window.onbeforeunload=function(){
//		evptr.sock.onclose();
//	}
	this.sock.onopen=function(){
		console.log(url+'/'+arguments.length+'/WS Initialize Success');
		if(evcb!=null) evcb(evptr);
	}
	this.sock.onmessage=function(event){
		console.log('WS message:'+event.data);
		evptr.set(JSON.parse(event.data));
	}
	this.sock.onerror=function(){
		console.log('WS Error!');
	}
	this.sock.onclose=function(){
		console.log('WS Connection Close!');
	}
}
WsShare.prototype={
	sock:null,
	send:function(str){
		var s=str;
		if(arguments.length==0) s=JSON.stringify(this);
		console.log('WSsend:'+s);
		this.sock.send(s);
	},
	sendObject:function(obj){
		var s=JSON.stringify(arguments.length>0? obj:this);
		this.send(s);
	},
	set:function(obj){
		for(var key in obj){
			var val=obj[key];
			if((typeof this[key])==='undefined'){
				console.log('WsShare.set undefined, so make it:'+key);
				this[key]=val;
			}
			else if((typeof this[key])==='function'){
				console.log('WsShare.set.call:'+key+'='+val);
				this[key](val);
			}
			else{
				console.log('WsShare.set.val:'+key+'='+val);
				this[key]=val;
			}
		}
	}
}
