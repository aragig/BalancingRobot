#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import time
from datetime import datetime
import smbus
from gpiozero import PWMLED
from gpiozero import Motor
from gpiozero import Robot
import sys
import threading
import termios
import socket


motorL = Motor(forward=12, backward=13)
motorR = Motor(forward=18, backward=19)
# robot = Robot(left=(18, 19), right=(12, 13))


GYRO_ADDR = 0x69
bus = smbus.SMBus(1)

# BMX055
# Data sheet -> https://www.mouser.jp/datasheet/2/783/BST-BMX055-DS000-1509552.pdf
# Acceleration address, 0x19
# Select PMU_Range register, 0x0F(15)
#       0x03(03)    Range = +/- 2g
bus.write_byte_data(0x19, 0x0F, 0x03)
# Select PMU_BW register, 0x10(16)
#       0x08(08)    Bandwidth = 7.81 Hz
bus.write_byte_data(0x19, 0x10, 0x08)
# Select PMU_LPW register, 0x11(17)
#       0x00(00)    Normal mode, Sleep duration = 0.5ms
bus.write_byte_data(0x19, 0x11, 0x00)

time.sleep(0.5)


# Gyro address, 0x69
# Select Range register, 0x0F(15)
#       0x04(04)    Full scale = +/- 125 degree/s
bus.write_byte_data(0x69, 0x0F, 0x04)
# Select Bandwidth register, 0x10(16)
#       0x07(07)    ODR = 100 Hz
bus.write_byte_data(0x69, 0x10, 0x07)
# Select LPM1 register, 0x11(17)
#       0x00(00)    Normal mode, Sleep duration = 2ms
bus.write_byte_data(0x69, 0x11, 0x00)
time.sleep(0.5)


def accl():
    xA = yA = zA = 0

    try:
        data = bus.read_i2c_block_data(0x19, 0x02, 6)
        # Convert the data to 12-bits
        xA = ((data[1] * 256) + (data[0] & 0xF0)) / 16
        if xA > 2047:
            xA -= 4096
        yA = ((data[3] * 256) + (data[2] & 0xF0)) / 16
        if yA > 2047:
            yA -= 4096
        zA = ((data[5] * 256) + (data[4] & 0xF0)) / 16
        if zA > 2047:
            zA -= 4096
    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))

    return xA, yA, zA


def gyro():
    xG = yG = zG = 0

    try:
        data = bus.read_i2c_block_data(GYRO_ADDR, 0x02, 6)
        # Convert the data
        xG = (data[1] * 256) + data[0]
        if xG > 32767:
            xG -= 65536

        yG = (data[3] * 256) + data[2]
        if yG > 32767:
            yG -= 65536

        zG = (data[5] * 256) + data[4]
        if zG > 32767:
            zG -= 65536

    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))

    return xG, yG, zG


def parseParams(rcvmsg):
    params = {"kc": 0.0, "kp": 0.0, "ki": 0.0,
              "kd": 0.0, "offsetAngle": 0.0}

    for q in rcvmsg.split('&'):
        key, value = q.split('=')
        if key == 'kp':
            params["kp"] = float(value)
        elif key == 'ki':
            params["ki"] = float(value)
        elif key == 'kd':
            params["kd"] = float(value)
        elif key == 'kc':
            params["kc"] = float(value)
        elif key == 'offsetAngle':
            params["offsetAngle"] = float(value)

    return params


class RobotJob(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.move = 0
        self.Kp = 42
        self.Ki = 315
        self.Kd = 180
        self.Kc = 0.9964
        self.offsetAngle = 1.43
        self.isEnd = False

    def forward(self, value):
        motorL.forward(abs(value))
        motorR.forward(abs(value))

    def backward(self, value):
        motorL.backward(abs(value))
        motorR.backward(abs(value))

    def brake(self):
        motorL.brake()
        motorR.brake()

    def balance(self, value):
        if value < 0:
            self.forward(value)
        elif value > 0:
            self.backward(value)
        else:
            self.brake()

    time.sleep(0.5)

    def setup(self):
        _gyro = 0
        _angle = 0
        for i in range(0, 100):
            xGyro, yGyro, zGyro = gyro()
            _gyro += xGyro

            xAccl, yAccl, zAccl = accl()
            _angle = np.arctan2(
                zAccl, yAccl) * 180 / 3.141592

        _gyro = _gyro / 100
        _angle = _angle / 100

        return _gyro, _angle

    def stop(self):
        self.isEnd = True

    def run(self):
        degree = 0
        i = 0
        lastErr = 0
        errSum = 0
        preTime = time.time()

        offsetGyro, xxxx = self.setup()
        offsetGyro = 0

        angle = 90 - self.offsetAngle
        angleGyro = angle

        while True:

            if self.isEnd:
                motorL.stop()
                motorR.stop()
                print("End robot loop")
                break

            xAccl, yAccl, zAccl = accl()
            xGyro, yGyro, zGyro = gyro()

            now = time.time()

            dt = (now - preTime)
            preTime = now

            angleAccl = np.arctan2(
                zAccl, yAccl) * 180 / 3.141592 - self.offsetAngle

            # Full scale = +/- 125 degree/s
            # 125 / 32766 = 0.003815
            xGyro *= -1
            dGyro = (xGyro) * 0.003815 * dt
            angleGyro += dGyro
            angle = self.Kc * (angle + dGyro) + (1 - self.Kc) * angleAccl
            # print "angleGyro=%f angleAccl=%f angle=%f" % (angleGyro, angleAccl, angle)

            # PID制御
            # Proportional=比例、Integral=積分、Differential=微分
            error = angle / 90 - 1  # P成分：傾き0～180度 → -1～1
            errSum += error * dt  # I成分
            dErr = (error - lastErr) / dt / 125  # D成分：角速度±125dps → -1～1
            u = self.Kp * error + self.Ki * errSum + self.Kd * dErr + self.move

            lastErr = error

            if u < -1.0:
                u = -1.0
            elif u > 1.0:
                u = 1.0

            self.balance(u)

            if i % 1000 == 0:
                print(self.Kp, self.Ki, self.Kd, self.offsetAngle)
            i += 1


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

                t = RobotJob()
                # スレッドをデーモンに設定し、メインスレッドの終了とともにデーモンスレッドも終了させる。
                t.setDaemon(True)
                t.start()
                preset_msg = "kp=%f&ki=%f&kd=%f&kc=%f&offsetAngle=%f" % (
                    t.Kp, t.Ki, t.Kd, t.Kc, t.offsetAngle)
                print(preset_msg)
                conn.sendall(preset_msg)

            elif rcvmsg.startswith("kp="):
                print("recived params")
                params = parseParams(rcvmsg)
                print(params)
                t.Kp = params["kp"]
                t.Ki = params["ki"]
                t.Kd = params["kd"]
                t.Kc = params["kc"]
                t.offsetAngle = params["offsetAngle"]

            elif rcvmsg.startswith("disconnect"):
                print("disconnect")

                t.stop()
                t.join()
                conn.sendall("disconnect")  # メッセージを返します
                conn.close()
                conn = None

    print('Disconnected')
