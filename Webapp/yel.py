from flask import Flask
from flask import request
from flask import render_template

import RPi.GPIO as GPIO
import time
import threading
import random
 

GPIO.setwarnings(False)

#can't catch flask app shutdown timing. so, cleaning at start.
GPIO.cleanup()

GPIO.setmode(GPIO.BCM)


# init super sonic sencor
trig = 17
echo = 27

GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)


#init motor driver
pin1 = 6
pin2 = 5
ENA = 12

pin3 = 19
pin4 = 13
ENB = 26

GPIO.setup(pin1,    GPIO.OUT)
GPIO.setup(pin2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

GPIO.setup(pin3,    GPIO.OUT)
GPIO.setup(pin4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)





def stop():
    GPIO.output(pin1,False)
    GPIO.output(pin2,False)
    GPIO.output(pin3,False)
    GPIO.output(pin4,False)


# init stop
stop()




speed_turn = 20
speed_go = 30
speed_back = 20

time_turn = 0.15
time_go = 0.8



pwmA = GPIO.PWM(ENA, 300)
pwmA.start(speed_go)

pwmB = GPIO.PWM(ENB, 300)
pwmB.start(speed_go)


g_dt = 100


def turn_left():
    pwmA.ChangeDutyCycle(speed_turn)
    pwmB.ChangeDutyCycle(speed_turn)
    GPIO.output(pin1,True)
    GPIO.output(pin2,False)
    GPIO.output(pin3,False)
    GPIO.output(pin4,True)    
    time.sleep(time_turn)
    stop()
    
def turn_right():
    pwmA.ChangeDutyCycle(speed_turn)
    pwmB.ChangeDutyCycle(speed_turn)
    GPIO.output(pin1,False)
    GPIO.output(pin2,True)
    GPIO.output(pin3,True)
    GPIO.output(pin4,False)    
    time.sleep(time_turn)
    stop()
    
    
def go_foreward():
    global g_dt
    #print "Distance : ", g_dt, "cm"
    if g_dt < 50 :
        return
        
    pwmA.ChangeDutyCycle(speed_go)
    pwmB.ChangeDutyCycle(speed_go)
    GPIO.output(pin1,False)
    GPIO.output(pin2,True)
    GPIO.output(pin3,False)
    GPIO.output(pin4,True)
    time.sleep(time_go)
    
def go_back():
    pwmA.ChangeDutyCycle(speed_back)    
    pwmB.ChangeDutyCycle(speed_back)
    GPIO.output(pin1,True)
    GPIO.output(pin2,False)
    GPIO.output(pin3,True)
    GPIO.output(pin4,False)    
    time.sleep(time_go)
    stop()
    
    
def get_distance_wall():
    try :
        GPIO.output(trig, False)
        time.sleep(0.5)

        GPIO.output(trig, True)
        time.sleep(0.00001)
        GPIO.output(trig, False)
            

        while GPIO.input(echo) == 0:
            pulse_start = time.time()

        while GPIO.input(echo) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17000
        distance = round(distance, 2)
        
        
        return distance
        
    except:
        pass
    
    
def loop_distance(nloop,nsec):
    global g_dt
    while True :
        dt_t = get_distance_wall() 
        if dt_t < 1000 :
            g_dt = dt_t
        #print "Distance : ", g_dt, "cm"           
        
    
def loop_driver(nloop,nsec):
    global g_dt
    global thread_auto_start

    #try:
    #turn_left()
    #turn_right()
    #go_foreward()
    #go_back()
    while True :
        #dt = get_distance_wall() 
        #print "Distance : ", dt, "cm"           
        dt = g_dt
        
        # less then 5cm thread end
        if dt < 5 or thread_auto_start == False:
            break
            
        # less then 20cm random left or right or back    
        if dt < 50 :
            r_val = random.randint(1,3)
            if r_val == 1 :
                turn_left()
            if r_val == 2 :
                turn_right()
            if r_val == 3 :
                go_back()
        else :
            go_foreward()            
            stop()
            time.sleep(0.5)
          
            
    stop()
    thread_auto_start = False
    #print "end drive thread"
                    
           


thread_auto_start = False
thread_driver = threading.Thread(target=loop_driver,args=(0,0))


thread_distance = threading.Thread(target=loop_distance,args=(0,0))
thread_distance.start()


# start web server
app = Flask(__name__, static_url_path='')


@app.route("/")
def hello():
    #return app.send_static_file("control_monitor.html")
    return render_template("control_monitor.html")
    #return "Hello World! Flask"
    
@app.route("/left")
def web_left():
    turn_left()
    return "ok"
    
@app.route("/right")
def web_right():
    turn_right()
    return "ok"
    
@app.route("/back")
def web_go_back():
    go_back()
    return "ok"
    
@app.route("/go")
def web_go_foreward():
    go_foreward()
    stop()
    return "ok"
    
    
@app.route("/auto_on")
def web_auto_on():
    global thread_auto_start
    global thread_driver
    
    if thread_auto_start == True :
        return "Ok"    
    
    thread_driver = threading.Thread(target=loop_driver,args=(0,0))
    #print "start thread"

    thread_auto_start = True
    thread_driver.start()    
    #thread_driver.join()
    return "ok"
    
@app.route("/auto_off")
def web_auto_off():
    global thread_auto_start
    global thread_driver
    
    if thread_auto_start == False :
        return "Ok"
        
    thread_auto_start = False
    thread_driver.join()
    return "ok"
    
@app.route("/shutdown")
def web_shutdown():
    
    #Clean GPIO
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
    
    func = request.environ.get('werkzeug.server.shutdown')
    if func is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    func()
    return "Shutting down..."
    
    

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8080, debug=True )
