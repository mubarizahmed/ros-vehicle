import cv2 as cv
import numpy as np

capture = cv.VideoCapture(1)
while True:
    isTrue, frame = capture.read()
    cv.imshow('Video', frame)
    gray1 = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    gray1 = cv.convertScaleAbs(gray1, alpha=1.1, beta=-50)
    # gray = cv.cvtColor(gray, cv.COLOR_BGR2HSV)
    gray = cv.inRange(gray1, 155, 255)
    blur = cv.GaussianBlur(gray, (5,5), cv.BORDER_DEFAULT)
    canny = cv.Canny(blur, 125, 150)
    cv.imshow('Canny Edges', canny)

    lines = cv.HoughLinesP(canny, 1, np.pi/180, 70, minLineLength=50, maxLineGap=10)

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv.line(frame, (x1,y1), (x2,y2), (0,255,0), 2)

    cv.imshow('Gray 1', gray1)
    cv.imshow('Taped Track', frame)
    cv.imshow('Grayscale', gray)

    if cv.waitKey(20) & 0xFF == ord('d'): # if the letter d is pressed
        break

capture.release()
cv.destroyAllWindows()
