import RPi.GPIO as GPIO
import serial
import time

# serial port and trans speed
ser = serial.Serial('/dev/ttyS0', 9600)

# Servo 모터 핀 번호
servo_pin = 14

# GPIO 모드 설정
GPIO.setmode(GPIO.BCM)

# Servo 모터 핀 설정
GPIO.setup(servo_pin, GPIO.OUT)

# PWM 주파수 설정 (일반적으로 50Hz 사용)
pwm_freq = 50

# PWM 객체 생성
pwm = GPIO.PWM(servo_pin, pwm_freq)

# PWM 신호를 이용하여 서보 모터 각도 제어
def set_angle(angle):
    duty_cycle = (angle / 18) + 2
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.3)  # 서보 모터가 제어되는 동안 일시적으로 정지

try:
    # PWM 시작
    pwm.start(0)

    while True:
        if ser.in_waiting > 0:  # 데이터 통신이 가능할 때
            data = ser.readline().decode().strip()  # 데이터 값 받아오기
            #data = str(data)  # 데이터 값 문자열 변환
            print(data)  # 데이터 값 쉘 출력

            if 'on' in data:  # on 값을 받아올 시 서보 모터 ON
                set_angle(90)  # 적절한 각도 값으로 서보 모터를 움직임

            elif 'off' in data:  # off 값을 받아올 시 서보 모터 OFF
                set_angle(0)  # 적절한 각도 값으로 서보 모터를 움직임

        # 추가적인 동작이 필요하다면 여기에 작성하면 됩니다.

finally:
    # 리소스 정리
    pwm.stop()
    GPIO.cleanup()