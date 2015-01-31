#!/usr/bin/env python

import socket
sock = socket.socket()
sock.connect(("192.168.4.1", 1234))
sock.send("bootloader")
sock.close()
