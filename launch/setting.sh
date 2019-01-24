#!/bin/bash

echo "Camera setting"
arv-tool-0.6 control GevSCPSPacketSize=9000
arv-tool-0.6 control -n SENTECH-17C8087 TriggerDelay=50
echo "Network setting"
