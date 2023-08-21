import pygame
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

#RIGHT_FORWARD = 6
#RIGHT_BACKWARD = 5
#RIGHT_PWM = 12
#LEFT_FORWARD = 19
#LEFT_BACKWARD = 13
#LEFT_PWM = 26

#DC_MOTOR
ENA = 12
IN1 = 5
IN2 = 6
ENB = 26
IN3 = 19
IN4 = 13

STOP = 0
FORWARD = 1
BACKWARD = 2

CH1 = 0
CH2 = 1		#두 개의 모터를 사용할 경우

#핀의 입출력
OUTPUT = 1
INPUT = 0

H = 1
L = 0

def setPinConfig(EN,INA,INB):
	GPIO.setwarnings(False)
	GPIO.setup(EN, GPIO.OUT)
	GPIO.setwarnings(False)
	GPIO.setup(INA, GPIO.OUT)
	GPIO.setwarnings(False)
	GPIO.setup(INB, GPIO.OUT)

	pwm = GPIO.PWM(EN,100)
	pwm.start(0)
	return pwm

def setMotorControl(pwm, INA, INB, speed, state):
	#모터 속도 제어
	pwm.ChangeDutyCycle(speed)

	#모터 방향 제어
	if state == FORWARD:
		GPIO.output(INA, H)
		GPIO.output(INB, L)

	elif state == BACKWARD:
		GPIO.output(INA, L)
		GPIO.output(INB, H)

	elif state == STOP:
		GPIO.output(INA, L)
		GPIO.output(INB, L)

def setMotor(ch, speed, state):   #모터 두개 사용 할 때 필요
	if ch == CH1:
		setMotorControl(pwmA, IN1, IN2, speed, state)
	else:
		setMotorControl(pwmB, IN3, IN4, speed, state)

pwmA = setPinConfig(ENA, IN1, IN2)
pwmB = setPinConfig(ENB, IN3, IN4)


#서보모터
servoPin = 23
SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3

GPIO.setup(servoPin, GPIO.OUT)

servo = GPIO.PWM(servoPin, 50)
servo.start(0)

def setServoPos(degree):
	"""if degree > 180:
		degree = 180"""

	if degree > 120:
		degree = 120
	if degree < 70:
		degree = 70


	duty = SERVO_MIN_DUTY + (degree*(SERVO_MAX_DUTY - SERVO_MIN_DUTY)/180.0)
	#print("Degree: {} to {}(duty)".format(degree,duty))

	servo.ChangeDutyCycle(duty)

def degree_converter_UtoServo(U):
	#degree = 3*(U+60)/2
	degree = -0.4167*U+95
	return degree


#pygame
pygame.init()
screen = pygame.display.set_mode((400,400))

while True:
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			pygame.quit()

		key = pygame.key.get_pressed()
		if key[pygame.K_w]:
			setMotor(CH2, 30, FORWARD)
			print("go forward")
		if key[pygame.K_s]:
			setMotor(CH2, 30, BACKWARD)
			print("go backward")
		if key[pygame.K_a]:
			print("turn left")
			U = -60
			degree = degree_converter_UtoServo(U)
			setServoPos(degree)
		if key[pygame.K_d]:
			print("turn right")
			U = 60
			degree = degree_converter_UtoServo(U)
			setServoPos(degree)
		if key[pygame.K_w] == 0 and key[pygame.K_s] == 0:
			setMotor(CH2, 0, STOP)
			print("stop")
		if key[pygame.K_a] == 0 and key[pygame.K_d] == 0:
			U = 0
			degree = degree_converter_UtoServo(U)
			setServoPos(degree)
			print("straight")

servo.stop()
setMotor(CH2, 0, STOP)
GPIO.cleanup()