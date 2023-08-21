import RPi.GPIO as GPIO
GPIO.setwarnings(False)
import time

GPIO.setmode(GPIO.BCM)

pulse_start = 0
pulse_end = 0

pinTrig1 = 17 # 초음파 센서 1의 Trig 핀 			# Center Usonic
pinEcho1 = 27 # 초음파 센서 1의 Echo 핀 
pinTrig2 = 14 # 초음파 센서 2의 Trig 핀 			# Right Usonic
pinEcho2 = 15 # 초음파 센서 2의 Echo 핀 
pinTrig3 = 16 # 초음파 센서 3의 Trig 핀 			# Left Usonic
pinEcho3 = 20 # 초음파 센서 3의 Echo 핀 

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(pinTrig1,GPIO.OUT)
GPIO.setup(pinEcho1,GPIO.IN)
GPIO.setup(pinTrig2,GPIO.OUT)
GPIO.setup(pinEcho2,GPIO.IN)
GPIO.setup(pinTrig3,GPIO.OUT)
GPIO.setup(pinEcho3,GPIO.IN)

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


#if __name__ == '__main__':
    # 핀 번호 설정
    #pinTrig1 = 17 # 초음파 센서 1의 Trig 핀 			# Center Usonic
    #pinEcho1 = 27 # 초음파 센서 1의 Echo 핀 
    #pinTrig2 = 14 # 초음파 센서 2의 Trig 핀 			# Right Usonic
    #pinEcho2 = 15 # 초음파 센서 2의 Echo 핀 
    #pinTrig3 = 16 # 초음파 센서 3의 Trig 핀 			# Left Usonic
    #pinEcho3 = 20 # 초음파 센서 3의 Echo 핀 

    #try:
        #while True:
# 초음파 센서로부터 거리 측정
def DISTANCE():
    distance1 = getDistance(pinTrig1, pinEcho1)
    distance2 = getDistance(pinTrig2, pinEcho2)
    distance3 = getDistance(pinTrig3, pinEcho3)
    k = int(0)
    
    print("Distance 1: %.2f cm" % distance1)
    print("Distance 2: %.2f cm" % distance2)
    print("Distance 3: %.2f cm" % distance3)

    #Check whether the distance is 50 cm
    if distance1 < 15 and distance2 < 15 and distance3 < 15 :
        #Forward 1 seconds
        print ("Stop"+ str(distance1) + str(distance2) + str(distance3) )
        rightMotor(0, 0, 0)
        leftMotor(0, 0, 0)
        k = 3
    else:
        #Left 1 seconds
        print ("Forward "+ str(distance1) + str(distance2) + str(distance3) )
        rightMotor(1, 0, 40)
        leftMotor(1, 0, 40)
        k = 0

#except KeyboardInterrupt:
    #pass

GPIO.cleanup()