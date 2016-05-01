#!/bin/bash

chmod 755 /var/run/screen

screen -l /dev/serial/by-path/pci-0000\:00\:14.0-usb-0\:1.1\:1.0 -s 9600

