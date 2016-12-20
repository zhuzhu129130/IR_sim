#/bin/bash

lsmod | grep "ftdi_sio" && sudo rmmod -f ftdi_sio

sudo ./main
