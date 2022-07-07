# https://www.geeksforgeeks.org/drawing-with-mouse-on-images-using-python-opencv/
import cv2

# some relevant params from main.py
XDIMENSION = YDIMENSION = 256       # Max number of nodes in x.y dirs (MUST BE A POWER OF 2!)
XSPACING = YSPACING = 10     # The spacing between nodes in x, y dir [meters]

# path to terrain image
img = cv2.imread("/Users/Oliver/Documents/CODING/Python_Prgms/WeBots_ElevationMap/webots_moose/protos/textures/TerrainFeaturesScaled.png")

# variables
ix = -1
iy = -1
drawing = False

# circle params
radius = 12
color = (255, 0, 0)
thickness = -1

# point coordinate storage
xs = []
ys = []

def draw_rectangle_with_drag(event, x, y, flags, param):
      
    global ix, iy, drawing, img
      
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix = x
        iy = y            
              
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            cv2.rectangle(img, pt1 =(ix, iy),
                          pt2 =(x, y),
                          color =(0, 255, 255),
                          thickness =-1)
      
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        cv2.rectangle(img, pt1 =(ix, iy),
                      pt2 =(x, y),
                      color =(0, 255, 255),
                      thickness =-1)

def placeDot(event, x, y, flags, param):
    global ix, iy, drawing, img
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix = x
        iy = y      
            
    elif event == cv2.EVENT_LBUTTONUP:
        if drawing == True:
            xpos = int( ix / (img.shape[1]/(XSPACING*XDIMENSION)) )
            ypos = (YDIMENSION*YSPACING) - int( iy / (img.shape[0]/(YSPACING*YDIMENSION)) )
            print( f"x,y: {xpos}, {ypos}" )
            xs.append( xpos )
            ys.append( ypos )
            cv2.circle( img,
                        center =(ix, iy),
                        radius=radius,
                        color=color,
                        thickness=thickness)

cv2.namedWindow(winname = "Elevation Selection")
cv2.setMouseCallback("Elevation Selection", placeDot)
  
while True:
    cv2.imshow("Elevation Selection", img)
    if cv2.waitKey(10) == 27:
        print(f"xs:\n{xs}\n\nys:\n{ys}")
        break
cv2.destroyAllWindows()