import cv2
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.preview_configuration.main.size=(640, 480)
picam2.preview_configuration.main.format="RGB888"
picam2.preview_configuration.align()
picam2.video_configuration.enable_raw(0)
picam2.start()

while(True):
    # Capture frame-by-frame
    frame = picam2.capture_array()
    print(frame)
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()