import cv2
import numpy as np
from pyzbar.pyzbar import decode

cam = cv2.VideoCapture(2)
cam.set(3,1920)
cam.set(4,1080)


while True:
    succes, frame = cam.read(0)
    if not succes:
        print("failed to grab frame")
        break
    cv2.imshow("test", frame)

    k = cv2.waitKey(1)

    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
    elif k%256 == 32:
        # SPACE pressed
        img_name = "./Photos/opencv_frame.png"
        cv2.imwrite(img_name, frame)

        image = cv2.imread('./Photos/opencv_frame.png')
        for barcode in decode(image):
            myData = barcode.data.decode('utf-8')
            print(myData)

        if myData == "Deksel 1":
            Deksel1_Open = True
        else:
            Deksel1_Open = False
        print(Deksel1_Open)







cam.release()

cv2.destroyAllWindows()