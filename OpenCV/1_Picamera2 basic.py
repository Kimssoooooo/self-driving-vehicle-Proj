import cv2
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.preview_configuration.main.size=(640, 480)
picam2.preview_configuration.main.format="RGB888"
picam2.preview_configuration.align()
picam2.video_configuration.enable_raw(0)
picam2.start()

while True:
    frame = picam2.capture_array()
    cv2.imshow("piCam", frame)
    if cv2.waitKey(1) == ord('q'):
        break
cv2.destroyAllWindows()