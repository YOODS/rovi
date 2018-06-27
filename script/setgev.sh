#!/bin/sh

arch=`arch`

if [ $arch = "aarch64" ]; then
  arv-tool-0.4 control GevSCPSPacketSize=9000 > /tmp/setgev.log
fi
