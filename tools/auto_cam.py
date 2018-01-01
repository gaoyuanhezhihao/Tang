from __future__ import print_function
import cv2
from time import time, sleep

freq = 5
intv = 1.0/freq

cap = cv2.VideoCapture(1)
cap.set(cv2.cv.CV_CAP_PROP_FPS, freq)
idx = 0
tm = time()
while True:
    gap = time() - tm
    print(gap)
    # if gap < intv:
        # sleep(intv-gap)
    tm = time()
    ret, im = cap.read()
    cv2.imshow("video", im)
    cv2.waitKey(1)
    cv2.imwrite("./vd_cap/%d.jpg" % idx, im)
    idx += 1
