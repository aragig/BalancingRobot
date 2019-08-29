#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket

if __name__ == "__main__":

    host = "192.168.100.136"
    port = 5555
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    #    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host, port))
    s.listen(1)  # キューの最大数を指定

    print('Waiting for connections...')

    while True:
        # 誰かがアクセスしてきたら、コネクションとアドレスを入れる
        conn, addr = s.accept()
        print('Connection is OK')
        while True:
            if conn is None:
                break
            rcvmsg = conn.recv(1024)

            if rcvmsg != '':
                print(rcvmsg)

            if rcvmsg.startswith("get preset"):
                print("started connection")
                preset_msg = "kp=%f&ki=%f&kd=%f&kc=%f&offsetAngle=%f" % (
                    10, 200, 4000, 9.6, 1.0)
                print(preset_msg)
                conn.sendall(preset_msg)

            elif rcvmsg.startswith("kp="):
                print("recived params")
                print(rcvmsg)

            elif rcvmsg.startswith("disconnect"):
                print("disconnect")

                conn.sendall("disconnect")  # メッセージを返します
                conn.close()
                conn = None

    print('Disconnected')
