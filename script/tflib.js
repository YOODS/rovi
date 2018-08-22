const ros = require('rosnodejs');
const geometry_msgs=ros.require('geometry_msgs').msg;

tflib.toRT=function(tf){
  let qx=tf.rotation.x;
  let qy=tf.rotation.y;
  let qz=tf.rotation.z;
  let qw=tf.rotation.w;
  let RT=new Array([
    [1-2*qy*qy-2*qz*qz,2*qx*qy-2*qz*qw,2*qx*qz+2*qy*qw,tf.translation.x],
    [2*qx*qy+2*qz*qw,1-2*qx*qx2-2*qz*qz,2*qy*qz-2*qx*qw,tf.translation.y],
    [2*qx*qz-2*qy*qw,2*qy*qz+2*qx*qw,1-2*qx*qx-2*qy*qy,tf.translation.z],
    [0,0,0,1]
  ]);
}
tflib.fromRT=function(rt){

module.exports=tflib
