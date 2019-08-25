#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import time
from datetime import datetime
import smbus
from gpiozero import PWMLED
from gpiozero import Motor

motorL = Motor(forward=18, backward=19)
motorR = Motor(forward=12, backward=13)

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


def move(value):
    if value < -1.0:
        value = -1.0
    elif value > 1.0:
        value = 1.0

    if value < 0:
        motorL.forward(abs(value))
        motorR.forward(abs(value))
    elif value > 0:
        motorL.backward(abs(value))
        motorR.backward(abs(value))
    else:
        motorL.stop()
        motorR.stop()


time.sleep(0.5)


def setup():
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


degree = 0
i = 0
lastErr = 0
errSum = 0
Kp = 30
Ki = 150
Kd = 150

preTime = time.time()

offsetGyro, offsetAngle = setup()
offsetGyro = 0
offsetAngle = 2
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
    u = Kp * error + Ki * errSum + Kd * dErr

    lastErr = error
    move(u)
