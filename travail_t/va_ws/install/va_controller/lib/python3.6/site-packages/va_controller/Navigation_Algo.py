#!/usr/bin/env python3
# coding=UTF-8
import math
from pylimo import limo
import time

limo=limo.LIMO()
#limo.EnableCommand()

def main():

    #limo=limo.LIMO()
    limo.EnableCommand()
    L=0.2

    Kphi=0.9
    Kbeta=1.2
    s=10.5
    Kalpha=s-Kphi
    odopresleft=limo.GetLeftWheelOdeom()
    odopresright=limo.GetRightWheelOdom()
    x_init=-1
    y_init=-1
    theta_init=math.pi
    x=x_init
    y=y_init
    theta=theta_init
    limo.SetMotionCommand(linear_vel=0,steering_angle=0)

    time.sleep(1)
    previoustime=time.time()
    while x<0.05 or y<0.05:
        rho=math.sqrt(pow(x,2)+pow(y,2))
        beta= - math.atan(y/x)
        alpha= - beta-theta

        w=Kbeta*beta+Kalpha*alpha
        v=Kphi*rho
        gamma=math.atan((L*w)/v)
        print(f"{x:.3f} {y:.3f} {gamma:.3f} {beta:.3f}")
        if gamma>0.488:
            gamma=0.488
        elif gamma<-0.488:
            gamma=-0.488
        if v>0.5:
            v=0.5
        
        dl=(limo.GetLeftWheelOdeom()-odopresleft)/1000.0
        dr=(limo.GetRightWheelOdom()-odopresright)/1000.0
        odopresleft=limo.GetLeftWheelOdeom()
        odopresright=limo.GetRightWheelOdom()
        
        limo.SetMotionCommand(linear_vel=v,steering_angle=gamma)
        v=limo.GetLinearVelocity()
        gamma=limo.GetSteeringAngle()

        lapse=time.time()-previoustime

        x_prime=v*math.cos(theta)
        y_prime=v*math.sin(theta)
        theta_prime=(v/L)*math.tan(gamma)
        
        x=x+x_prime*lapse
        y=y+y_prime*lapse
        theta=theta+theta_prime*lapse

        previoustime=time.time()

main()
