#!/usr/bin/python

"Device Discovery utility for GigE cameras"

import socket, struct, os, sys, binascii
from collections import deque

GVCP = 3956
HOSTIP  = "192.168.1.240"
CAMIP   = "192.168.1.250"
CAMMAC  = "00111cf01676"
CAMMASK = "255.255.255.0"
CAMGW   = "0.0.0.0"

def gige(typ, op, seq, data = ""):
    sz = len(data)
    return struct.pack(">HHHH", typ, op, sz, seq) + data

def forceip(CAMMAC, CAMIP, CAMMASK, CAMGW):
    mac = int(CAMMAC, 16)
    mach = (mac >> 32) & 0xffff
    macl = mac & 0xffffffff
    ips = socket.inet_aton(CAMIP)
    mask = socket.inet_aton(CAMMASK)
    gw = socket.inet_aton(CAMGW)
    fmt = ">xxHIxxxxxxxxxxxx4sxxxxxxxxxxxx4sxxxxxxxxxxxx4s"
    data = struct.pack(fmt, mach, macl, ips, mask, gw)
    return gige(0x4201, 0x0004, 0xffff, data)

argq=deque(sys.argv[1:])
argc=len(sys.argv)-1
while(argq):
	arg=argq.popleft()
	if arg[0:2]=='-h':
		HOSTIP=argq.popleft() if len(arg)==2 else arg[2:]
		print "HostIP   :",HOSTIP
	elif arg[0:2]=='-c':
		CAMIP=argq.popleft() if len(arg)==2 else arg[2:]
		print "CAMERA IP:",CAMIP
	elif arg[0:2]=='-m':
		CAMMASK=argq.popleft() if len(arg)==2 else arg[2:]
		print "IP MASK  :",CAMMASK
	elif arg[0:2]=='-g':
		CAMGW=argq.popleft() if len(arg)==2 else arg[2:]
		print "GATEWAY  :",CAMGW
	else:
		print "Invarild Argument:",arg

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.settimeout(1.0)
sock.bind((HOSTIP, GVCP))

discovery = gige(0x4201, 0x0002, 0xffff)
sock.sendto(discovery, ("255.255.255.255", GVCP))
ack=sock.recv(256);
CAMMAC=binascii.hexlify(ack[18:24])
ip    =struct.unpack(">BBBB",ack[44:48])
subnet=struct.unpack(">BBBB",ack[60:64])
gw    =struct.unpack(">BBBB",ack[76:80])
mfname=struct.unpack(">32s",ack[80:112])
mdname=struct.unpack(">32s",ack[112:144])
dver  =struct.unpack(">32s",ack[144:176])
msinfo=struct.unpack(">48s",ack[176:224])
snum  =struct.unpack(">16s",ack[224:240])
udname=struct.unpack(">16s",ack[240:256])
print "DISCOVERY ACK", struct.unpack(">HHHH", ack[0:8])
print "Ver(MAJ/MIN) ", struct.unpack(">HH", ack[8:12])
print "MAC Address  ", CAMMAC
print "Current IP   ", ip
print " Subnet Mask ", subnet
print " Gateway     ", gw 
print "Manufacturer Name: ", mfname
print "Model Name       : ", mdname
print "Device Version   : ", dver
print "Manufacturer info: ", msinfo
print "Serial Number    : ", snum
print "User-Defined Name: ", udname

setup = forceip(CAMMAC, CAMIP, CAMMASK, CAMGW)
sock.sendto(setup, ("255.255.255.255", GVCP))
print "camera replies", struct.unpack(">HHHH", sock.recv(8))
os.system("ping -c 5 -s 32 %s" % CAMIP)
print "You might have to run this script twice..."
