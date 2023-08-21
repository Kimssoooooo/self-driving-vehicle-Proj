import cv2
from picamera2 import Picamera2
from picamera2 import Picamera2, Preview
import RPi.GPIO as GPIO
import time
from time import sleep

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

def setMotor(ch, speed, state):
    # 모터 두 개 사용할 때 필요
    if ch == CH1:
        setMotorControl(pwmA, IN1, IN2, speed, state)
    else:
        setMotorControl(pwmB, IN3, IN4, speed, state)

pwmA = setPinConfig(ENA, IN1, IN2)
pwmB = setPinConfig(ENB, IN3, IN4)

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

# 초음파 센서
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
    
    
    frame = picam2.capture_array()
    picam2.rotation = 180
    frame = cv2.flip(frame, -1)
    frame = cv2.resize(frame, dsize = (640, 360), interpolation = cv2.INTER_AREA)
    cv2.imshow('test', frame)
    
    #picam2 = Picamera2()
    #picam2.preview_configuration.main.size=(640, 480)
    #picam2.preview_configuration.main.format="RGB888"
    #picam2.preview_configuration.align()
    #picam2.video_configuration.enable_raw(0)
    #config = picam2.create_preview_configuration()

    #camera_config = picam2.create_still_configuration(main={"size": (1920, 1080)}, lores={"size": (640, 480)}, display="lores")
    #picam2.configure(config)
    #picam2.start_preview(Preview.QTGL)
    #picam2.start()
    time.sleep(1)
    picam2.stop_preview()
    
    #picam2.capture_file("/home/pi/Pictures/Ptest-python.jpg")
    #filepath = "/home/pi/Project/Final/picture/img"
    i=0
    carstate = "stop"

    while( picam2.is_open == True ):

        keyValue = cv2.waitKey(10)
        #print(str(keyValue))
        
        if keyValue == ord('q'):
            break
        #up
        elif keyValue == ord('w'):
            print("go")
            carstate = "go"
        #down
        elif keyValue == ord('s'):
            print("down")
            carstate = "stop"
        #left
        elif keyValue == ord('a'):
            print("left")
            carstate = "left"
        #right
        elif keyValue == ord('d'):
            print("right")
            carstate = "right"

        #ret , image = camera.read()
        #image = cv2.flip(image,-1)
        #cv2.imshow('original',image)
        
        
        #height, _ , _ = image.shape
        #save_image = image[int(height/2):,:,:]
        #save_image = cv2.cvtColor(save_image, cv2.COLOR_BGR2YUV)
        #save_image = cv2.GaussianBlur(save_image, (3,3),0)
        #save_image = cv2.realize(save_image, (200,66))
        #cv2.imshow('save',save_image)
        
        if carstate =="left":
            #picam2.capture_file("%s_%05d_03d.png" %(filepath, i, 45), image)
            picam2.capture_file("/home/pi/Pictures/proj/Ptest_%05d_%03d.jpg'" %(i, 45) )

            #cv2.imwrite("%s_%05d_03d.png" %(filepath, i, 45), image)
            i += 1
        elif carstate =="right":
            #cv2.imwrite("%s_%05d_03d.png" %(filepath, i, 135), image)
            picam2.capture_file("/home/pi/Pictures/proj/Ptest_%05d_%03d.jpg" %(i, 135) )
            i += 1
            
        elif carstate =="go":
            #cv2.imwrite("%s_%05d_03d.png" %(filepath, i, 90), image)
            picam2.capture_file("/home/pi/Pictures/proj/Ptest_%05d_%03d.jpg" %(i, 90) )
            i += 1

        time.sleep(1.0)
        
        
        if keyValue == 27: #esc is end
            break

    cv2.destroyAllWindows()

if __name__ =='__main__':
    main()
    GPIO.cleanup()