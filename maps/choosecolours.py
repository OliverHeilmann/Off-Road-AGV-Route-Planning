"""
Credit to Berak for this handy colour range picker program!

His profile: https://answers.opencv.org/users/2130/berak/
Source Code: https://answers.opencv.org/question/134248/how-to-define-the-lower-and-upper-range-of-a-color/?answer=134284#post-id-134284

"""

import cv2
import numpy as np
import sys
sys.path.append('./maps')     # add 'search' directory to path


PATH_TO_IMAGE = "webots_moose/protos/textures/TerrainFeatures.png"

image_hsv = None   # global ;(
pixel = (20,60,80) # some default

# mouse callback function
def pick_color(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN:
        pixel = image_hsv[y,x]
        rgbimg = cv2.cvtColor(image_hsv, cv2.COLOR_HSV2RGB)
        rgbpixel = rgbimg[y,x]

        #you might want to adjust the ranges(+-10, etc):
        pct = 15
        upper =  np.array([pixel[0] + pct, pixel[1] + pct, pixel[2] + 40])
        lower =  np.array([pixel[0] - pct, pixel[1] - pct, pixel[2] - 40])
        
        # -------------MidRGB-----------MidHSV-------LowerHSV-------UpperHSV-----
        print(f"{list(rgbpixel)}, {list(pixel)}, {list(lower)}, {list(upper)}")

        image_mask = cv2.inRange(image_hsv,lower,upper)
        cv2.imshow("mask",image_mask)

def main( imgPath = "" ):
    global image_hsv, pixel # so we can use it in mouse callback

    image_src = cv2.imread( imgPath )  # pick.py my.png
    image_src = cv2.resize(image_src, (1024, 1024), interpolation = cv2.INTER_AREA )
    if image_src is None:
        print ("the image read is None............")
        return
    cv2.imshow("bgr",image_src)

    ## NEW ##
    cv2.namedWindow('hsv')
    cv2.setMouseCallback('hsv', pick_color)

    # now click into the hsv img , and look at values:
    image_hsv = cv2.cvtColor(image_src,cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv",image_hsv)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__=='__main__':
    main( imgPath = PATH_TO_IMAGE )