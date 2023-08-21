import os
import time
import cv2
import numpy as np
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.preview_configuration.main.size=(1920,1080)
picam2.preview_configuration.main.format="RGB888"
picam2.preview_configuration.align()
picam2.video_configuration.enable_raw(0)
picam2.start()

frame = picam2.capture_array()
frame = cv2.flip(frame, -1)
frame = cv2.resize(frame, dsize = (640, 360), interpolation = cv2.INTER_AREA)

def capture_and_save():
    # 캡쳐한 이미지를 저장할 디렉토리 생성
        i=898
        
        try:
            while True:
                # 현재 시간으로 파일명 생성
                #file_name = f"/home/pi/Desktop/captures/capture_%05d.jpg" %(i)
                keyValue = cv2.waitKey(10)
                # 카메라에서 이미지 캡쳐
                picam2.capture_file(f"/home/pi/Desktop/angle_capture/capture_%d_-65.jpg" %(i))
                i += 1
                
                
                # 1초마다 캡쳐
                cv2.imshow('capture', frame)
                time.sleep(0.1)
                
                if keyValue == 27: #esc is end
                    break
            
            cv2.destroyAllWindows()
        
        finally:
            pass

if __name__ == "__main__":
    capture_and_save()