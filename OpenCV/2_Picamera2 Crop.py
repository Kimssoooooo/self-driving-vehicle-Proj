import cv2
from picamera2 import Picamera2
import numpy as np

def main():
    picam2 = Picamera2()
    picam2.preview_configuration.main.size=(1920,1080)
    picam2.preview_configuration.main.format="RGB888"
    picam2.preview_configuration.align()
    picam2.video_configuration.enable_raw(0)
    picam2.start()

    while( picam2.is_open == True ):
        frame = picam2.capture_array()
        frame = cv2.flip(frame, -1)
        frame = cv2.resize(frame, dsize = (640, 360), interpolation = cv2.INTER_AREA)
        cv2.imshow(' normal' , frame)
        
        crop_img = frame[100:360, 0:640]
        cv2.imshow('crop', crop_img)
        
        if cv2.waitKey(1) == ord('q'):
            breakq

    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()