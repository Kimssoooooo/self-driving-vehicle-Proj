import cv2
from picamera2 import Picamera2
import numpy as np
import pygame
import RPi.GPIO as GPIO
import time

pygame.init()
        
screen = pygame.display.set_mode((400, 400))

    
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

# Pin의 입출력
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
    # 모터 속도 제어
    pwm.ChangeDutyCycle(speed)

    # 모터 방향 제어
    if state == FORWARD:
        GPIO.output(INA, H)
        GPIO.output(INB, L)
    elif state == BACKWARD:
        GPIO.output(INA, L)
        GPIO.output(INB, H)
    elif state == STOP:
        GPIO.output(INA, L)
        GPIO.output(INB, L)

pwmA = setPinConfig(ENA, IN1, IN2)
pwmB = setPinConfig(ENB, IN3, IN4)

def setMotor(ch, speed, state):
    # 모터 두 개 사용할 때 필요
    if ch == CH1:
        setMotorControl(pwmA, IN1, IN2, speed, state)
    else:
        setMotorControl(pwmB, IN3, IN4, speed, state)

    #pwmA = setPinConfig(ENA, IN1, IN2)
    #pwmB = setPinConfig(ENB, IN3, IN4)

# 서보모터
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
    degree = -0.4167 * U + 95
    return degree

if __name__ == '__main__':
    picam2 = Picamera2()
    picam2.preview_configuration.main.size=(640, 360)
    picam2.preview_configuration.main.format="RGB888"
    picam2.preview_configuration.align()
    picam2.video_configuration.enable_raw(0)
    picam2.start()

    while( picam2.is_open == True ):
        frame = picam2.capture_array()
        #frame = cv2.flip(frame, -1)
        #cv2.imshow(' normal' , frame)
        
        crop_img = frame[100:360, 0:640]
        
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        ret, thresh1 = cv2.threshold(blur, 150,255, cv2.THRESH_BINARY_INV)
        
        mask = cv2.erode(thresh1, None, iterations = 2)
        
        mask = cv2.dilate(mask, None, iterations = 2)
        
        #cv2.imshow('mask', mask)
        
        contours, hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)
        
        if len(contours) > 0:
            c = max(contours, key = cv2.contourArea)
            M = cv2.moments(c)
            
            cx = int(M ['m10'] / M ['m00'])
            cy = int(M ['m01'] / M ['m00'])
            
            cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
            cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)
            
            cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)
            #print(cx)
            
        
        # Pygame
        pygame.init()
        
        screen = pygame.display.set_mode((400, 400))
        

        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        raise KeyboardInterrupt

                key = pygame.key.get_pressed()

                if key[pygame.K_w] and key[pygame.K_d]:
                    setMotor(CH1, 50, FORWARD)
                    setMotor(CH2, 50, FORWARD)
                    print("right and forward")
                    print(cx)
                    U = -65
                    degree = degree_converter_UtoServo(U)
                    setServoPos(degree)
                elif key[pygame.K_w] and key[pygame.K_a]:
                    setMotor(CH1, 50, FORWARD)
                    setMotor(CH2, 50, FORWARD)
                    print("left and forward")
                    print(cx)
                    U = 65
                    degree = degree_converter_UtoServo(U)
                    setServoPos(degree)
                elif key[pygame.K_s] and key[pygame.K_a]:
                    setMotor(CH1, 50, BACKWARD)
                    setMotor(CH2, 50, BACKWARD)
                    print("left and backward")
                    print(cx)
                    U = 65
                    degree = degree_converter_UtoServo(U)
                    setServoPos(degree)
                elif key[pygame.K_s] and key[pygame.K_d]:
                    setMotor(CH1, 50, BACKWARD)
                    setMotor(CH2, 50, BACKWARD)
                    print("left and backward")
                    print(cx)
                    U = -60
                    degree = degree_converter_UtoServo(U)
                    setServoPos(degree)
                elif key[pygame.K_w]:
                    setMotor(CH1, 50, FORWARD)
                    setMotor(CH2, 50, FORWARD)
                    print("go forward")
                    print(cx)
                elif key[pygame.K_s]:
                    setMotor(CH1, 50, BACKWARD)
                    setMotor(CH2, 50, BACKWARD)
                    print("go backward")
                    print(cx)
                elif key[pygame.K_d]:
                    print("turn right")
                    print(cx)
                    U = -65
                    degree = degree_converter_UtoServo(U)
                    setServoPos(degree)
                elif key[pygame.K_a]:
                    print("turn left")
                    print(cx)
                    U = 65
                    degree = degree_converter_UtoServo(U)
                    setServoPos(degree)
                elif key[pygame.K_w] == 0 and key[pygame.K_s] == 0 and key[pygame.K_d] == 0 and key[pygame.K_a] == 0:
                    setMotor(CH1, 0, STOP)
                    setMotor(CH2, 0, STOP)
                    print("stop")
                    print(cx)
                    U = 0
                    degree = degree_converter_UtoServo(U)
                    setServoPos(degree)
                elif key[pygame.K_d] == 0 and key[pygame.K_a] == 0:
                    setMotor(CH1, 50, STOP)
                    setMotor(CH2, 50, STOP)
                    print("straight")
                    print(cx)
                    U = 0
                    degree = degree_converter_UtoServo(U)
                    setServoPos(degree)

        except KeyboardInterrupt:
            print("Exiting program")
            
        finally:
            servo.stop()
            setMotor(CH1, 0, STOP)
            setMotor(CH2, 0, STOP)
            GPIO.cleanup()
        
            
        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
    