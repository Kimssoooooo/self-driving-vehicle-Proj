import RPi.GPIO as GPIO
GPIO.setwarnings(False)
import time

GPIO.setmode(GPIO.BCM)

pulse_start = 0
pulse_end = 0

def getDistance(trigPin, echoPin):
    GPIO.setup(trigPin, GPIO.OUT)
    GPIO.setup(echoPin, GPIO.IN)

    GPIO.output(trigPin, False)
    time.sleep(0.5)

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

if __name__ == '__main__':
    # 핀 번호 설정
    pinTrig1 = 4 # 초음파 센서 1의 Trig 핀 			# Center Usonic
    pinEcho1 = 17 # 초음파 센서 1의 Echo 핀 
    pinTrig2 = 2 # 초음파 센서 2의 Trig 핀 			# Right Usonic
    pinEcho2 = 3 # 초음파 센서 2의 Echo 핀 
    pinTrig3 = 12 # 초음파 센서 3의 Trig 핀 			# Left Usonic
    pinEcho3 = 16 # 초음파 센서 3의 Echo 핀 
    #pinRed = 8
    #pinGreen = 9
    #pinBlue = 7

    try:
        while True:
            # 초음파 센서로부터 거리 측정
            dist1 = getDistance(pinTrig1, pinEcho1)
            dist2 = getDistance(pinTrig2, pinEcho2)
            dist3 = getDistance(pinTrig3, pinEcho3)

            print("Distance 1: %.2f cm" % dist1)
            print("Distance 2: %.2f cm" % dist2)
            print("Distance 3: %.2f cm" % dist3)

            # 거리에 따른 LED 제어
            #GPIO.setup(pinRed, GPIO.OUT)
            #GPIO.setup(pinGreen, GPIO.OUT)
            #GPIO.setup(pinBlue, GPIO.OUT)

            #GPIO.output(pinRed, False)
            #GPIO.output(pinGreen, False)
            #GPIO.output(pinBlue, False)

            #if dist1 <= 0.3:
                #GPIO.output(pinRed, True)
            #elif dist2 <= 0.3:
               # GPIO.output(pinGreen, True)
           # elif dist3 <= 0.3:
                #GPIO.output(pinBlue, True)

            #time.sleep(1)

    except KeyboardInterrupt:
        pass

GPIO.cleanup()