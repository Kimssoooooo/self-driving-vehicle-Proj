import cv2

cap = cv2.VideoCapture("/home/pi/Videos/CVtest_objectdetector.mp4")

#object detection from stable camera
object_detector = cv2.createBackgroundSubtractorMOG2()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    print(frame)
    
    mask = object_detector.apply(frame)

    # Display the resulting frame
    cv2.imshow("Frame",frame)
    cv2.imshow("Mask",mask)
    
    key = cv2.waitkey(30)
    if Key == 27:
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()