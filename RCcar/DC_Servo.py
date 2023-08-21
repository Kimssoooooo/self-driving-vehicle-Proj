import RPi.GPIO as GPIO
import time
import Ultrasonic as US

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

# Pygame
#screen = pygame.display.set_mode((400, 400))
k = int()

try:
    while True:
        US.DISTANCE()
        
        if k == 0:
            setMotor(CH1, 30, FORWARD)
            setMotor(CH2, 30, FORWARD)
            print("go forward")
        #elif key[pygame.K_w] and key[pygame.K_a]:
            #setMotor(CH1, 30, FORWARD)
            #setMotor(CH2, 30, FORWARD)
            #print("left and forward")
            #U = 60
            #degree = degree_converter_UtoServo(U)
            #setServoPos(degree)
        #elif key[pygame.K_s] and key[pygame.K_a]:
            #setMotor(CH1, 30, BACKWARD)
            #setMotor(CH2, 30, BACKWARD)
            #print("left and backward")
            #U = 60
            #degree = degree_converter_UtoServo(U)
            #setServoPos(degree)
        #elif key[pygame.K_s] and key[pygame.K_d]:
            #setMotor(CH1, 30, BACKWARD)
            #setMotor(CH2, 30, BACKWARD)
            #print("left and backward")
            #U = -60
            #degree = degree_converter_UtoServo(U)
            #setServoPos(degree)
        #elif key[pygame.K_w]:
            #setMotor(CH1, 30, FORWARD)
            #setMotor(CH2, 30, FORWARD)
            #print("go forward")
        #elif key[pygame.K_s]:
            #setMotor(CH1, 30, BACKWARD)
            #setMotor(CH2, 30, BACKWARD)
            #print("go backward")
        elif k == 1:
            print("turn right")
            U = -60
            degree = degree_converter_UtoServo(U)
            setServoPos(degree)
        elif k == 2:
            print("turn left")
            U = 60
            degree = degree_converter_UtoServo(U)
            setServoPos(degree)
        elif k == 3:
            setMotor(CH1, 0, STOP)
            setMotor(CH2, 0, STOP)
            print("stop")
            U = 0
            degree = degree_converter_UtoServo(U)
            setServoPos(degree)
        #elif key[pygame.K_d] == 0 and key[pygame.K_a] == 0:
            #setMotor(CH1, 30, STOP)
            #setMotor(CH2, 30, STOP)
            #print("straight")
            #U = 0
            #degree = degree_converter_UtoServo(U)
            #setServoPos(degree)

except KeyboardInterrupt:
    print("Exiting program")

finally:
    servo.stop()
    setMotor(CH1, 0, STOP)
    setMotor(CH2, 0, STOP)
    GPIO.cleanup()