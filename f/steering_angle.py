import cv2
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model

#def img_preprocess(image):
    #height, _, _ = image.shape
    #image = image[int(height/2) : , : , :]
    #image = cv2.cvtColor(image,cv2.COLOR_BGR2YUV)
    #image = cv2.GaussianBlur(image,(3,3),0)
    #image = cv2.resize(image,(640, 360))
    #image = image / 255
    #return image


def sub(frame):
    #picam2 = Picamera2()
    #picam2.preview_configuration.main.size=(1920,1080)
    #picam2.preview_configuration.main.format="RGB888"
    #picam2.preview_configuration.align()
    #picam2.video_configuration.enable_raw(0)
    #picam2.start()
    #camera = cv2.VideoCapture(0)
    #camera.set(3, 640)
    #camera.set(4, 360)
    model_path = f'/home/pi/Project/Final/lane_navigation_final.h5'
    model = load_model (model_path)

    #carState = "stop"

    #while(picam2.is_open == True):

    #keyValue = cv2.waitKey(10)

    #if keyValue == ord('q') :
        #break

    #image = picam2.read()
    #image = picam2.capture_array()
    #image = cv2.flip(image, -1)
    #image = cv2.resize(image, dsize = (480, 270), interpolation = cv2.INTER_AREA)
    #image = cv2.flip(image,0)
    #cv2.imshow('original', image)

    #preprocessed = img_preprocess(image)
    #cv2.imshow('pre', preprocessed)

    X = np.asarray([frame])
    steering_angle = (((model.predict(X)[0])-600)/11)
    print("predict angle: ", steering_angle)
    #return steering_angle
    #if steering_angle >= -30 and steering_angle <= 30 :
        #print("go")
    #if steering_angle > 30 :
        #print("left")
    #if steering_angle <= -30 :
        #print("right")


    cv2.destroyAllWindows()

if __name__ == '__sub__':
    sub()
    GPIO.cleanup()
















