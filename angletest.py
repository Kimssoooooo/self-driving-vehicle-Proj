import cv2
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import time
from time import sleep
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
#import steering_angle as angle

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# DC_MOTOR
ENA = 26
IN1 = 19
IN2 = 13
ENB = 12
IN3 = 6
IN4 = 5

STOP = 0
FORWARD = 1
BACKWARD = 2

CH1 = 0
CH2 = 1

# INPUT and OUTPUT Pin def
OUTPUT = 1
INPUT = 0

H = 1
L = 0

def setPinConfig(EN, INA, INB):
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)

    pwm = GPIO.PWM(EN, 100)
    pwm.start(0)
    return pwm

def setMotorControl(pwm, INA, INB, speed, state):
    # Control the speed
    pwm.ChangeDutyCycle(speed)

    # Control the direction for DC motor
    if state == FORWARD:
        GPIO.output(INA, H)
        GPIO.output(INB, L)
    elif state == BACKWARD:
        GPIO.output(INA, L)
        GPIO.output(INB, H)
    elif state == STOP:
        GPIO.output(INA, L)
        GPIO.output(INB, L)

def setMotor(ch, speed, state):
    # When using two motor, need to this code
    if ch == CH1:
        setMotorControl(pwmA, IN1, IN2, speed, state)
    else:
        setMotorControl(pwmB, IN3, IN4, speed, state)

pwmA = setPinConfig(ENA, IN1, IN2)
pwmB = setPinConfig(ENB, IN3, IN4)

# Servo motor
servoPin = 23
SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3

GPIO.setup(servoPin, GPIO.OUT)

servo = GPIO.PWM(servoPin, 50)
servo.start(0)

def setServoPos(degree):
    if degree > 120:
        degree = 120
    if degree < 70:
        degree = 70

    duty = SERVO_MIN_DUTY + (degree * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180.0)
    servo.ChangeDutyCycle(duty)

def degree_converter_UtoServo(U):
    degree = int(-0.4167 * U + 95)
    return degree

# UltraSonic sensor
pulse_start = 0
pulse_end = 0

pinTrig1 = 17 # UltraSonic 1 Trig Pin 			# Center Usonic
pinEcho1 = 27 # UltraSonic 1 Echo Pin 
pinTrig2 = 14 # UltraSonic 2 Trig Pin 			# Right Usonic
pinEcho2 = 15 # UltraSonic 2 Echo Pin 
pinTrig3 = 16 # UltraSonic 3 Trig Pin 			# Left Usonic
pinEcho3 = 20 # UltraSonic 3 Echo Pin 

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(pinTrig1,GPIO.OUT)
GPIO.setup(pinEcho1,GPIO.IN)
GPIO.setup(pinTrig2,GPIO.OUT)
GPIO.setup(pinEcho2,GPIO.IN)
GPIO.setup(pinTrig3,GPIO.OUT)
GPIO.setup(pinEcho3,GPIO.IN)

def getDistance(trigPin, echoPin):
    GPIO.setup(trigPin, GPIO.OUT)
    GPIO.setup(echoPin, GPIO.IN)

    GPIO.output(trigPin, False)
    time.sleep(0.1)

    GPIO.output(trigPin, True)
    time.sleep(0.00001)
    GPIO.output(trigPin, False)

    while GPIO.input(echoPin) == 0:
        pulse_start = time.time()

    while GPIO.input(echoPin) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17000
    distance = round(distance, 2)

    return distance

#Right Motor Control
def rightMotor(forward, backward, pwm):
    GPIO.output(RIGHT_FORWARD,forward)
    GPIO.output(RIGHT_BACKWARD,backward)
    RIGHT_MOTOR.ChangeDutyCycle(pwm)

#Left Motor Control
def leftMotor(forward, backward, pwm):
    GPIO.output(LEFT_FORWARD,forward)
    GPIO.output(LEFT_BACKWARD,backward)
    LEFT_MOTOR.ChangeDutyCycle(pwm)

def main():
    picam2 = Picamera2()
    picam2.preview_configuration.main.size=(1920,1080)
    picam2.preview_configuration.main.format="RGB888"
    picam2.preview_configuration.align()
    picam2.video_configuration.enable_raw(0)
    picam2.start()
    model_path = f'/home/pi/Project/Final/lane_navigation_final.h5'
    model = load_model (model_path)
    
    
    while( picam2.is_open == True ):
        
        distance1 = getDistance(pinTrig1, pinEcho1)
        distance2 = getDistance(pinTrig2, pinEcho2)
        distance3 = getDistance(pinTrig3, pinEcho3)
        
        frame = picam2.capture_array()
        frame = cv2.flip(frame, -1)
        frame = cv2.resize(frame, dsize = (480, 270), interpolation = cv2.INTER_AREA)
        frame = frame / 255
        cv2.imshow(' normal' , frame)
        
        crop_img = frame[0:270, 0:480]
        
        #gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        
        blur = cv2.GaussianBlur(frame, (5, 5), 110)
        cv2.imshow('blur', blur)
        
        #ret, thresh1 = cv2.threshold(blur, 130,255, cv2.THRESH_BINARY_INV)
        
        #mask = cv2.erode(thresh1, None, iterations = 2)
        
        #mask = cv2.dilate(mask, None, iterations = 2)
        
        #cv2.imshow('mask', mask)
        
        #contours, hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)
        
        X = np.asarray([blur])
        steering_angle = int((model.predict(X)[0]))  #-700)/13
        
        
        #if len(contours) > 0:
            #c = max(contours, key = cv2.contourArea)
            #M = cv2.moments(c)
            
            #cx = int(M ['m10'] / M ['m00'])
            #cy = int(M ['m01'] / M ['m00'])
            
            
        keyValue = cv2.waitKey(8)
        
        if distance1 < 15 or distance2 < 15 or distance3 < 15 or keyValue == ord('s'):
            setMotor(CH1, 0, STOP)
            setMotor(CH2, 0, STOP)
            U = 0
            degree = degree_converter_UtoServo(U)
            setServoPos(degree)
            print("stop")
            print("predict angle: ", steering_angle)
            #print(cx)
            print("Distance 1: %.2f cm" % distance1)
            print("Distance 2: %.2f cm" % distance2)
            print("Distance 3: %.2f cm" % distance3)
            
        else:
            setMotor(CH1, 23, FORWARD)
            setMotor(CH2, 23, FORWARD)
            U = (steering_angle * 1.6)
            degree = degree_converter_UtoServo(U)
            setServoPos(degree)
            print("go")
            print("predict angle: ", steering_angle)
            #print(cx)
            print("Distance 1: %.2f cm" % distance1)
            print("Distance 2: %.2f cm" % distance2)
            print("Distance 3: %.2f cm" % distance3)
            
        #if  cx <= 270 : #280 <= cx <= 300:
            #setMotor(CH1, 20, FORWARD)
            #setMotor(CH2, 20, FORWARD)
            #print("left and forward")
            #print(cx)
            #U = 65
            #degree = degree_converter_UtoServo(U)
            #setServoPos(degree)
            #print("Distance 1: %.2f cm" % distance1)
            #print("Distance 2: %.2f cm" % distance2)
            #print("Distance 3: %.2f cm" % distance3)
            #sleep(2)
            
        #elif 380 <= cx : #450 >= cx >= 430 :
            #setMotor(CH1, 20, FORWARD)
            #setMotor(CH2, 20, FORWARD)
            #print("right and forward")
            #print(cx)
            #U = -65
            #degree = degree_converter_UtoServo(U)
            #setServoPos(degree)
            #print("Distance 1: %.2f cm" % distance1)
            #print("Distance 2: %.2f cm" % distance2)
            #print("Distance 3: %.2f cm" % distance3)
            #sleep(2)
            
        #elif 270 < cx < 380 :
            #setMotor(CH1, 22, FORWARD)
            #setMotor(CH2, 22, FORWARD)
            #print("go forward")
            #print(cx)
            #U = 0
            #degree = degree_converter_UtoServo(U)
            #setServoPos(degree)
            #print("Distance 1: %.2f cm" % distance1)
            #print("Distance 2: %.2f cm" % distance2)
            #print("Distance 3: %.2f cm" % distance3)
            
        #else :
            #setMotor(CH1, 0, STOP)
            #setMotor(CH2, 0, STOP)
            #print("stop")
            #print(cx)
            #U = 0
            #degree = degree_converter_UtoServo(U)
            #setServoPos(degree)
            #print("Distance 1: %.2f cm" % distance1)
            #print("Distance 2: %.2f cm" % distance2)
            #print("Distance 3: %.2f cm" % distance3)
           
            
        
        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()
    GPIO.cleanup()




