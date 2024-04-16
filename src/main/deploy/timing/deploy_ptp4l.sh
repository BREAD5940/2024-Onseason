#!/usr/bin/env bash

scp ptp4l admin@10.59.40.2:/usr/bin/ptp4l
ssh admin@10.59.40.2 chmod +x /usr/bin/ptp4l

scp pmc admin@10.59.40.2:/usr/bin/pmc
ssh admin@10.59.40.2 chmod +x /usr/bin/pmc

scp ptp4linit admin@10.59.40.2:/etc/init.d/ptp4linit
ssh admin@10.59.40.2 chmod +x /etc/init.d/ptp4linit
ssh admin@10.59.40.2 ln -s ../init.d/ptp4linit /etc/rc5.d/S80ptp4linit
