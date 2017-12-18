from __future__ import print_function
import cv2
cap = cv2.VideoCapture(0)

idx = 0
while True:
    ret, im = cap.read()
    cv2.imshow("video", im)
    key = cv2.waitKey(10)
    if key is not -1:
        print("key%s" % repr(key))
    if key == 1114085:  # press ESC to exit.
        break
    if key == 1048608: # press SPACe to save.
        print("write img")
        cv2.imwrite("./vd_cap/vi_result%s.jpg" % idx, im)
        idx += 1
