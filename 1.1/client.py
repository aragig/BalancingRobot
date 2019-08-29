# -*- coding:utf-8 -*-
import socket
import time

host = "192.168.100.136"
port = 5555

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((host, port))

massage = "p=100&i=50&d=0"

while True:
    client.send(massage.encode('utf-8'))

    response = client.recv(4096)  # レシーブは適当な2の累乗にします（大きすぎるとダメ）

    print(response)

    time.sleep(1)

print 'Disconnected'
