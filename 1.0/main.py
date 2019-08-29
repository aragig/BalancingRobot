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

motorL = Motor(forward=18, backward=19)
motorR = Motor(forward=12, backward=13)
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
        print"I/O error({0}): {1}".format(e.errno, e.strerror)

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
        print "I/O error({0}): {1}".format(e.errno, e.strerror)

    return xG, yG, zG


class RobotJob(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.move = 0

    def forward(self, value):
        motorL.forward(abs(value))
        motorR.forward(abs(value))

    def backward(self, value):
        motorL.backward(abs(value))
        motorR.backward(abs(value))

    def stop(self):
        motorL.brake()
        motorR.brake()

    def balance(self, value):
        if value < 0:
            self.forward(value)
        elif value > 0:
            self.backward(value)
        else:
            self.stop()

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

    def run(self):
        degree = 0
        i = 0
        lastErr = 0
        errSum = 0
        Kp = 45
        Ki = 250
        Kd = 220
        preTime = time.time()

        offsetGyro, offsetAngle = self.setup()
        offsetGyro = 0
        offsetAngle = 1.5

        angle = 90 - offsetAngle
        angleGyro = angle

        while True:

            xAccl, yAccl, zAccl = accl()
            xGyro, yGyro, zGyro = gyro()

            now = time.time()

            dt = (now - preTime)
            preTime = now

            angleAccl = np.arctan2(
                zAccl, yAccl) * 180 / 3.141592 - offsetAngle

            K = 0.996

            # Full scale = +/- 125 degree/s
            # 125 / 32766 = 0.003815
            xGyro *= -1
            dGyro = (xGyro) * 0.003815 * dt
            angleGyro += dGyro
            angle = K * (angle + dGyro) + (1 - K) * angleAccl
            # print "angleGyro=%f angleAccl=%f angle=%f" % (angleGyro, angleAccl, angle)

            # PID制御
            # Proportional=比例、Integral=積分、Differential=微分
            error = angle / 90 - 1  # P成分：傾き0～180度 → -1～1
            errSum += error * dt  # I成分
            dErr = (error - lastErr) / dt / 125  # D成分：角速度±125dps → -1～1
            u = Kp * error + Ki * errSum + Kd * dErr + self.move

            lastErr = error

            if u < -1.0:
                u = -1.0
            elif u > 1.0:
                u = 1.0

            self.balance(u)

            if i % 1000 == 0:
                print(u)
            i += 1


if __name__ == "__main__":

    t = RobotJob()
    # スレッドをデーモンに設定し、メインスレッドの終了とともにデーモンスレッドも終了させる。
    t.setDaemon(True)
    t.start()

    # 標準入力のファイルディスクリプタを取得
    fd = sys.stdin.fileno()

    # fdの端末属性をゲットする
    # oldとnewには同じものが入る。
    # newに変更を加えて、適応する
    # oldは、後で元に戻すため
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)

    # new[3]はlflags
    # ICANON(カノニカルモードのフラグ)を外す
    new[3] &= ~termios.ICANON
    # ECHO(入力された文字を表示するか否かのフラグ)を外す
    new[3] &= ~termios.ECHO

    while True:
        try:
            # 書き換えたnewをfdに適応する
            termios.tcsetattr(fd, termios.TCSANOW, new)
            # キーボードから入力を受ける。
            # lfalgsが書き換えられているので、エンターを押さなくても次に進む。echoもしない
            c = sys.stdin.read(1)
            if c == 'j':  # 前進
                t.move = 0.5
                print("前進")
                time.sleep(3)
                t.move = 0
            # elif c == 'k':  # 後進
            #     t.move = -0.3
            #     print("後進")
            elif c == 's':  # Stop
                t.move = 0
                print("s")
            elif c == 'q':
                t.kill_flag = True

        finally:
            # fdの属性を元に戻す
            # 具体的にはICANONとECHOが元に戻る
            termios.tcsetattr(fd, termios.TCSANOW, old)
