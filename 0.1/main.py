#!/usr/bin/env python

# -*- coding: utf-8 -*-
import smbus
from gpiozero import PWMLED
from time import sleep
import numpy as np
from gpiozero import Motor

# FIN_L = PWMLED(18, frequency=80000)
# RIN_L = PWMLED(19, frequency=80000)
# FIN_R = PWMLED(12, frequency=80000)
# RIN_R = PWMLED(13, frequency=80000)
motorL = Motor(forward=18, backward=19)
motorR = Motor(forward=12, backward=13)

GYRO_ADDR = 0x69
bus = smbus.SMBus(1)

# BMX055 Accl address, 0x19
# Select PMU_Range register, 0x0F(15)
#       0x03(03)    Range = +/- 2g
bus.write_byte_data(0x19, 0x0F, 0x03)
# BMX055 Accl address, 0x19
# Select PMU_BW register, 0x10(16)
#       0x08(08)    Bandwidth = 7.81 Hz
bus.write_byte_data(0x19, 0x10, 0x08)
# BMX055 Accl address, 0x19
# Select PMU_LPW register, 0x11(17)
#       0x00(00)    Normal mode, Sleep duration = 0.5ms
bus.write_byte_data(0x19, 0x11, 0x00)

sleep(0.5)


# BMX055 Gyro address, 0x69
# Select Range register, 0x0F(15)
#       0x04(04)    Full scale = +/- 125 degree/s
bus.write_byte_data(0x69, 0x0F, 0x04)
# BMX055 Gyro address, 0x69
# Select Bandwidth register, 0x10(16)
#       0x07(07)    ODR = 100 Hz
bus.write_byte_data(0x69, 0x10, 0x07)
# BMX055 Gyro address, 0x69
# Select LPM1 register, 0x11(17)
#       0x00(00)    Normal mode, Sleep duration = 2ms
bus.write_byte_data(0x69, 0x11, 0x00)
sleep(0.5)


def accl():
    xAccl = 0
    yAccl = 0
    zAccl = 0

    try:
        data = bus.read_i2c_block_data(0x19, 0x02, 6)
        # Convert the data to 12-bits
        xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16
        if xAccl > 2047:
            xAccl -= 4096
        yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16
        if yAccl > 2047:
            yAccl -= 4096
        zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16
        if zAccl > 2047:
            zAccl -= 4096
    except IOError as e:
        print"I/O error({0}): {1}".format(e.errno, e.strerror)

    return xAccl, yAccl, zAccl


def gyro():

    xGyro = 0
    yGyro = 0
    zGyro = 0

    try:
        data = bus.read_i2c_block_data(GYRO_ADDR, 0x02, 6)
        # Convert the data
        xGyro = (data[1] * 256) + data[0]
        if xGyro > 32767:
            xGyro -= 65536

        yGyro = (data[3] * 256) + data[2]
        if yGyro > 32767:
            yGyro -= 65536

        zGyro = (data[5] * 256) + data[4]
        if zGyro > 32767:
            zGyro -= 65536

        # 125 / 32766 = 0.003815
        # 2000 / 32766 = 0.061039
        # 1000 / 32766 = 0.122078
        xGyro = xGyro * 0.003815  # Full scale = +/- 125 degree/s
        yGyro = yGyro * 0.003815  # Full scale = +/- 125 degree/s
        zGyro = zGyro * 0.003815  # Full scale = +/- 125 degree/s
    except IOError as e:
        print "I/O error({0}): {1}".format(e.errno, e.strerror)

    return xGyro, yGyro, zGyro


def forward():
    motorL.forward(0.9)
    motorR.forward(0.9)
    # FIN_L.value = 1
    # RIN_L.value = 0
    # FIN_R.value = 1
    # RIN_R.value = 0


def backward():
    motorL.backward(0.9)
    motorR.backward(0.9)
    # FIN_L.value = 0
    # RIN_L.value = 1
    # FIN_R.value = 0
    # RIN_R.value = 1


def brake():
    motorL.stop()
    motorR.stop()
    # FIN_L.value = 1
    # RIN_L.value = 1
    # FIN_R.value = 1
    # RIN_R.value = 1


degree = 0
# pregx = 0
i = 0
angle = 90
dt = 0.01
k = 0.9

while True:

    xAccl, yAccl, zAccl = accl()

    xGyro, yGyro, zGyro = gyro()
    # degree += xGyro * dt
    # degree += (pregx + xGyro) * 0.1 / 2
    # pregx = xGyro

    # if zAccl > 1000:
    #     zAccl = 1000
    # elif zAccl < -1000:
    #     zAccl = -1000

    angleAccel = np.arctan2(
        zAccl, xAccl) * 180 / 3.141592
#    angleAccel = np.arcsin(zAccl / 1000.0) * 180.0 / 3.141592
    # print "%f" % (angleAccel)

    angle = k * (angle + xGyro * dt) + (1 - k) * angleAccel
    # print "%f %f %f" % (xAccl, yAccl, zAccl)

    if angle > 90.2:
        forward()
    elif angle < 89.8:
        backward()
    else:
        brake()

    # print(
    #     'Gyro= ({:>+13.4f}, {:>+13.4f}, {:>+13.4f})'.format(xGyro, yGyro, zGyro))
    sleep(dt)
