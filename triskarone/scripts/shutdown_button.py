#!/bin/python

import RPi.GPIO as GPIO
import os
import time

BUTTON=12

GPIO.setmode(GPIO.BOARD)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

while True:
    GPIO.wait_for_edge(BUTTON, GPIO.FALLING)
    t = time.time()
    GPIO.wait_for_edge(BUTTON, GPIO.RISING)
    if time.time() - t > 2:
	os.system("sudo shutdown -h now")

