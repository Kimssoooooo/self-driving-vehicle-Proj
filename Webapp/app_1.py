from flask import Flask, render_template, request
import RPi.GPIO as GPIO
import time

app = Flask(__name__)

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


# 웹 페이지를 위한 라우트를 정의합니다.
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/forward')
def forward():
    # DC 모터 제어 코드
    setMotor(CH1, 30, FORWARD)
    setMotor(CH2, 30, FORWARD)
    # 서보 모터 제어 코드
    return 'go forward'

@app.route('/backward')
def backward():
    # DC 모터 제어 코드
    setMotor(CH1, 30, BACKWARD)
    setMotor(CH2, 30, BACKWARD)
    # 서보 모터 제어 코드
    return 'go backward'

@app.route('/left')
def left():
    # 서보 모터 제어 코드
    U = 60
    degree = degree_converter_UtoServo(U)
    setServoPos(degree)
    return 'turn left'

@app.route('/right')
def right():
    # 서보 모터 제어 코드
    U = -60
    degree = degree_converter_UtoServo(U)
    setServoPos(degree)
    return 'turn right'

@app.route('/stop')
def stop():
    # DC 모터 제어 코드
    setMotor(CH1, 0, STOP)
    setMotor(CH2, 0, STOP)
    # 서보 모터 제어 코드
    U = 0
    degree = degree_converter_UtoServo(U)
    setServoPos(degree)
    return 'stop'


if __name__ == '__main__':
    # 서버를 실행합니다.
    app.run(host='0.0.0.0', port=8080)