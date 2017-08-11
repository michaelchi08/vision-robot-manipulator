#import the necessary modules
import freenect
import cv2
import numpy as np
# import sys


#function to get RGB image from kinect
def mouse_callback(event,x,y,flags,param):
  r = img[y][x][2]
  g = img[y][x][1]
  b = img[y][x][0]
  h = hsv[y][x][0]
  s = hsv[y][x][1]
  v = hsv[y][x][2]
  output_rgb = "R:%d, G:%d, B:%d " % (r, g, b)
  output_hsv = "H:%d, S:%d, V:%d" % (h, s, v)
  tmp = img.copy()
  cv2.putText(tmp,output_rgb, (10,20), font, 0.5, (0,0,0))
  cv2.putText(tmp,output_hsv, (10,40), font, 0.5, (0,0,0))
  cv2.imshow('window', tmp)
  if event == cv2.EVENT_LBUTTONDOWN:
      print "hsv: (%d, %d, %d)" % (h,s,v)


def get_video():
    array,_ = freenect.sync_get_video()
    array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    return array
 
if __name__ == "__main__":

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.namedWindow("window",1)
    cv2.setMouseCallback("window",mouse_callback)
    img = get_video()
    

    while 1:
        #get a frame from RGB camera
        img = get_video()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)       
        cv2.imshow('window', img)
        # cv2.imshow('window', img)

        k = cv2.waitKey(20) & 0xFF
        if k == 27:
            break
    cv2.destroyAllWindows()


