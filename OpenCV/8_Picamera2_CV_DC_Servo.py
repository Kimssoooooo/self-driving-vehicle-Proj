import cv2
import numpy as np
from picamera2 import Picamera2
import RPi.GPIO as GPIO

#SET DC MOTOR
PWMA = 26
AIN1 = 19
AIN2 = 13

PWMB = 12
BIN1 = 6
BIN2 = 5

def moter_go(speed):
    L_Motor.ChangeDutyCycle(speed)
    GPIO_output(AIN2, True) #AIN2
    GPIO_output(AIN1, False) #AIN1
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, True) #BIN2
    GPIO.output(BIN1, False) #BIN1

def moter_right(speed):
    L_Motor.ChangeDutyCycle(speed)
    GPIO_output(AIN2, True) #AIN2
    GPIO_output(AIN1, False) #AIN1
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, False) #BIN2
    GPIO.output(BIN1, True) #BIN1

def moter_left(speed):
    L_Motor.ChangeDutyCycle(speed)
    GPIO_output(AIN2, False) #AIN2
    GPIO_output(AIN1, True) #AIN1
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, True) #BIN2
    GPIO.output(BIN1, False) #BIN1

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(PWMA, GPIO.OUT)

GPIO.setup(BIN2, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)

L_Motor = GPIO.PWM(PWMA, 100)
L_Motor.start(0)

L_Motor = GPIO.PWM(PWMB, 100)
L_Motor.start(0)


#SET SERVO MOTOR
servoPin = 23
SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3

GPIO.setup(servoPin, GPIO.OUT)

servo = GPIO.PWM(servoPin, 50)
servo.start(0)

def setServoPos(degree):
    #GPIO.setup(servoPin, GPIO.OUT)
    """if degree > 180:
        degree = 180"""

    if degree > 120:
        degree = 120
    if degree < 70:
        degree = 70

    duty = SERVO_MIN_DUTY + (degree*(SERVO_MAX_DUTY - SERVO_MIN_DUTY)/180.0)
    #print("Degree: {} to {}(dety)".format(degree,duty))
    servo.ChangeDutyCycle(duty)

def degree_converter_UtoServo(U):
    #degree = 3*(U+60)/2
    degree = -0.4167*U+95
    return degree


# MAIN FUNCTION
def main():
    picam2 = Picamera2()
    picam2.preview_configuration.main.size=(640, 480)
    picam2.preview_configuration.main.format="RGB888"
    picam2.preview_configuration.align()
    picam2.video_configuration.enable_raw(0)
    picam2.start()
    
    while( picam2.is_open == True ):
        frame = picam2.capture_array()
        #frame = cv2.flip(frame, -1)
        cv2.imshow(' normal' , frame)
        
        crop_img = frame[100:480, 0:640]
        
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        ret, thresh1 = cv2.threshold(blur, 130,255, cv2.THRESH_BINARY_INV)
        
        mask = cv2.erode(thresh1, None, iterations = 2)
        
        mask = cv2.dilate(mask, None, iterations = 2)
        
        cv2.imshow('mask', mask)
        
        contours, hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)
        
        if len(contours) > 0:
            c = max(contours, key = cv2.contourArea)
            M = cv2.moments(c)
            
            cx = int(M ['m10'] / M ['m00'])
            cy = int(M ['m01'] / M ['m00'])
            
            #cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
            #cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)
            
            #cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)
            
            if cx <= 180:
                print('Turn Left')
            
            elif cx >= 420:
                print('Turn Right')
                
            else:
                print('go')
            
        
        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()
    GPIO.cleanup()