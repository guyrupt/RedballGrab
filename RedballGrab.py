#!/usr/bin/env python
from smbus import SMBus
from pynput import keyboard
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import sys

cap = cv2.VideoCapture(0)
_, frame=cap.read()
rows, cols, _ = frame.shape
x_medium = int(cols/2)
center = int(cols/2)
y_line = int(rows/16*14)
y_medium=int(rows/2)
deg=90
addr = 0x8 
bus = SMBus(1) 
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11,GPIO.OUT)
GPIO.setup(12,GPIO.OUT)
servo1 = GPIO.PWM(11,50)
servo2 = GPIO.PWM(12,50)
servo1.start(0)
servo2.start(0)
servo1.ChangeDutyCycle(2+(180/18))
servo1.ChangeDutyCycle(2+(90/18))
low_red = np.array([156, 155, 84])
high_red = np.array([180, 255, 255])


while True:
    _, frame=cap.read()
    hsv_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    
    
    red_mask = cv2.inRange(hsv_frame, low_red, high_red)
    
    
    contours, _= cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse = True)
    for cnt in contours:
        (x,y,w,h)=cv2.boundingRect(cnt)
        
        x_medium=int((x+x+w)/2)
        y_medium=int((h/2)+y)
        break
        
    cv2.line(frame, (x_medium,0), (x_medium,480), (0,255,0),2)
    cv2.line(frame, (0,y_line), (960,y_line), (255,0,0),2)
    cv2.line(frame, (0,y_medium), (960,y_medium), (255,0,0),2)
    if len(contours) > 0:
        M = cv2.moments(contours[0])
        area_ball=M["m00"]
        print(area_ball)
    else:
        print("No ball detected")
        bus.write_byte(addr, int(2))
    if x_medium < center+50  and y_medium<=y_line:
        bus.write_byte(addr, int(4))
        print('turn left')
    elif x_medium > center  and y_medium<=y_line:
        bus.write_byte(addr, int(5))
        print('turn right')
    elif y_medium<=y_line:
        bus.write_byte(addr, int(1))
        print('straight')
    elif y_medium>y_line and x_medium > cols*0.25:
        bus.write_byte(addr, int(2))
        print('stop')
        servo1.ChangeDutyCycle(2+(90/18))
        servo2.ChangeDutyCycle(2+(180/18))
    if y_medium<=y_line:
        servo1.ChangeDutyCycle(2+(180/18))
        servo2.ChangeDutyCycle(2+(90/18))
    cv2.imshow("Frame",frame)
    
    key=cv2.waitKey(1)
    
    if key==113 or key ==81:
        break


    time.sleep(0.015)
    bus.write_byte(addr, int(2))
    time.sleep(0.05)
    
    

print('quit successfully')
bus.write_byte(addr, int(2))
servo1.ChangeDutyCycle(2+(180/18))
servo2.ChangeDutyCycle(2+(90/18))
time.sleep(0.015)
servo1.ChangeDutyCycle(0)
servo1.stop()
time.sleep(0.015)
servo2.ChangeDutyCycle(0)
servo2.stop()
GPIO.cleanup()
cv2.destroyAllWindows()  
cap.release()
    
