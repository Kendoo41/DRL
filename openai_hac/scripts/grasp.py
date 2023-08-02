#!/usr/bin/env python


import RPi.GPIO as GPIO
import time

class Gripper:
    def __init__(self):
        GPIO.setwarnings(False)
        self.output_pins = {
        'JETSON_XAVIER': 18,
        'JETSON_NANO': 33,
        'JETSON_NX': 33,
        'CLARA_AGX_XAVIER': 18,
        'JETSON_TX2_NX': 32,
        'JETSON_ORIN': 18,
        }
        self.output_pin = self.output_pins.get(GPIO.model, None)
        if self.output_pin is None:
            raise Exception('PWM not supported on this board')
        self.en_pin = 12
        self.dir_pin = 16
         # Pin Setup:
        # Board pin-numbering scheme
        GPIO.setmode(GPIO.BOARD)
        # set pin as an output pin with optional initial state of HIGH
        GPIO.setup(self.en_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.dir_pin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.output_pin, GPIO.OUT, initial=GPIO.HIGH)
        self.p = GPIO.PWM(self.output_pin, 800) #
        self.val = 50   #25%  

    def Grasp(self):
        try:
            
            GPIO.output(self.en_pin, GPIO.HIGH)
            GPIO.output(self.dir_pin, GPIO.HIGH)
            self.p.start(self.val)
            start = time.time()
            time.process_time()
            t = 0
            while t < 0.8:
                t = time.time() - start
                time.sleep(0.01)
            self.p.stop()
        finally:
            GPIO.output(self.en_pin, GPIO.LOW)
            #GPIO.cleanup()
    def Drop(self):
        try:
            GPIO.output(self.en_pin, GPIO.HIGH)
            GPIO.output(self.dir_pin, GPIO.LOW)
            self.p.start(self.val)
            start = time.time()
            time.process_time()
            t = 0
            while t < 0.8:
                t = time.time() - start
                time.sleep(0.01)
            self.p.stop()
        finally:
            GPIO.output(self.en_pin, GPIO.LOW)
            #GPIO.cleanup()
    def clean(self):
        GPIO.cleanup()
if __name__=="__main__":
    a = Gripper()
    a.Grasp()
    time.sleep(5)
    a.Drop()
    a.clean()