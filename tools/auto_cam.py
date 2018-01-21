from __future__ import print_function
import cv2
from time import time, sleep
import os
import glob

freq = 5
intv = 1.0/freq

cap = cv2.VideoCapture(0)
# cap.set(cv2.cv.CV_CAP_PROP_FPS, freq)
cap.set(3, 1280)
cap.set(4, 720)
idx = 0
tm = time()
dest = os.path.expanduser("~/car/data/vd_cap/")
map(os.unlink, glob.glob(dest+"*"))
tmpl = dest+"%d.jpg"
print(tmpl)
while True:
    gap = time() - tm
    print(gap)
    # if gap < intv:
        # sleep(intv-gap)
    tm = time()
    ret, im = cap.read()
    cv2.imshow("video", im)
    cv2.waitKey(1)
    cv2.imwrite(tmpl% idx, im)
    idx += 1
