import cv2 as cv
import numpy as np

# Load the cascade
face_cascade = cv.CascadeClassifier('haarcascade-facefrontal-default.xml')

# To capture video from webcam.
capture = cv.VideoCapture(1)
while True:
    isTrue, frame = capture.read()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
  
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)

    for (x,y,w,h) in faces:
        cv.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)

    cv.imshow('Face Detection', frame)

    if cv.waitKey(5) & 0xFF==ord('d'): # if the letter d is pressed, destroy window
        break

capture.release()
cv.destroyAllWindows()
