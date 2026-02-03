#!/usr/bin/env python3
# coding=UTF-8
from pylimo import limo
import time
limo=limo.LIMO()


while True:

####Valeurs des capteurs####
    print("Linear velocity")
    print(limo.GetLinearVelocity())
    print("Steering angle")
    print(limo.GetSteeringAngle())
    print("Lateral Velocity")
    print(limo.GetLateralVelocity())
    print("Left wheel odometer")
    print(limo.GetLeftWheelOdeom())
    print("Right wheel odometer")
    print(limo.GetRightWheelOdom())
    print("IMU acceleration")
    print(limo.GetIMUAccelData())
    print("Gyroscope data")
    print(limo.GetIMUGyroData())
    print("Pitch angle")
    print(limo.GetIMUPichData())
    print("Roll angle")
    print(limo.GetIMURollData())

####Commande des actinneurs(moteurs)####
    limo.SetMotionCommand(linear_vel=0.5,steering_angle=0)
    time.sleep(0.1)

