import cv2
import time
import RPi.GPIO as GPIO
#Set GPIO BCM(Broadcom SoC) pin number

#center
TRIG1 = 17
ECHO1 = 27
#right
TRIG2 = 14
ECHO2 = 15
#left
TRIG3 = 16
ECHO3 = 20

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(TRIG1,GPIO.OUT)
GPIO.setup(ECHO1,GPIO.IN)
GPIO.setup(TRIG2,GPIO.OUT)
GPIO.setup(ECHO2,GPIO.IN)
GPIO.setup(TRIG3,GPIO.OUT)
GPIO.setup(ECHO3,GPIO.IN)

RIGHT_FORWARD = 6
RIGHT_BACKWARD = 5
RIGHT_PWM = 12
LEFT_FORWARD = 19
LEFT_BACKWARD = 13
LEFT_PWM = 26

GPIO.setup(RIGHT_FORWARD,GPIO.OUT)
GPIO.setup(RIGHT_BACKWARD,GPIO.OUT)
GPIO.setup(RIGHT_PWM,GPIO.OUT)
GPIO.output(RIGHT_PWM, 0)
RIGHT_MOTOR = GPIO.PWM(RIGHT_PWM, 100)
RIGHT_MOTOR.start(0)
RIGHT_MOTOR.ChangeDutyCycle(0)

GPIO.setup(LEFT_FORWARD,GPIO.OUT)
GPIO.setup(LEFT_BACKWARD,GPIO.OUT)
GPIO.setup(LEFT_PWM,GPIO.OUT)
GPIO.output(LEFT_PWM, 0)
LEFT_MOTOR = GPIO.PWM(LEFT_PWM, 100)
LEFT_MOTOR.start(0)
LEFT_MOTOR.ChangeDutyCycle(0)

#Get distance from HC-SR04
def getDistance(TRIG,ECHO):
    GPIO.output(TRIG, False)
    time.sleep(0.1)
    
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    
    pulse_start = time.time()
    pulse_end = time.time()
    
    #When the ECHO is LOW, get the purse start time
    while GPIO.input(ECHO)==0:
        pulse_start = time.time()
        
    #When the ECHO is HIGN, get the purse end time
    while GPIO.input(ECHO)==1:
        pulse_end = time.time()
        
        #Get pulse duration time
        pulse_duration = pulse_end - pulse_start
        
        #Multiply pulse duration by 17150 to get distance and round
        distance = pulse_duration * 17150
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

if __name__ == '__main__':
    try:
        while True:
            distance1 = getDistance(TRIG1,ECHO1)
            print ("Distance1:", distance1)
            distance2 = getDistance(TRIG2,ECHO2)
            print ("Distance2:", distance2)
            distance3 = getDistance(TRIG3,ECHO3)
            print ("Distance3:", distance3)
            
            time.sleep(0.1)
            
            #Check whether the distance is 50 cm
            if distance1 < 15 and distance2 < 15 and distance3 < 15 :
                #Forward 1 seconds
                print ("Stop"+ str(distance1) + str(distance2) + str(distance3) )
                rightMotor(0, 0, 0)
                leftMotor(0, 0, 0)
            else:
                #Left 1 seconds
                print ("Forward "+ str(distance1) + str(distance2) + str(distance3) )
                rightMotor(1, 0, 40)
                leftMotor(1, 0, 40)

    except KeyboardInterrupt:
        print ('KeyboardInterrupt exception is caught')
    else:
        print ('No exceptions are caught')