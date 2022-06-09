
# sudo python3 -m pip install pybluez

import socket

emac = '00:1B:10:60:4C:9E'
port = 3 # not sure tbh

s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

s.connect((emac, port))

s.send(bytes('hello!\n', 'UTF-8'))
