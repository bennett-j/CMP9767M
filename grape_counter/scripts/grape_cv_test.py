import cv2
import os
from matplotlib import pyplot as plt

im_name = 'grapes2.png'
path = os.path.join(os.path.expanduser('~'), 'cmp9767_ws/src/CMP9767/grape_counter', im_name)
im = cv2.imread(path)
im = cv2.resize(im, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)

cv2.imshow('Raw Image', im)

# Convert BGR to HSV
hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

'''
 # bgr and hsv histograms 
#https://docs.opencv.org/4.x/d1/db7/tutorial_py_histogram_begins.html
color = ('b','g','r')
for i,col in enumerate(color):
    histr = cv2.calcHist([im],[i],None,[256],[0,256])
    plt.plot(histr,color = col)
    plt.xlim([0,256])
plt.show(block=False) # ensures window doesn't close? blocking behaviour? omit or put at end? show(block=False)?



color = ('c','m','y')
for i,col in enumerate(color):
    histr = cv2.calcHist([hsv],[i],None,[256],[0,256])
    plt.plot(histr,color = col)
    plt.xlim([0,256])
plt.show()
'''
'''
#                        b  g  r
mask1 = cv2.inRange(im, (0, 0, 130), (255, 255, 150)) # pixels have any b,g value but specific r value
mask2 = cv2.inRange(im, (0, 0, 160), (255, 255, 200))
cv2.imshow("mask1", mask1)
cv2.imshow("mask2", mask2)
'''

hsv_planes = cv2.split(hsv)
cv2.imshow("h", hsv_planes[0])
cv2.imshow("s", hsv_planes[1])
cv2.imshow("v", hsv_planes[2])

path = os.path.join(os.path.expanduser('~'), 'cmp9767_ws/src/CMP9767/grape_counter', 'grapes_hsv.png')
cv2.imwrite(path, hsv)



# mask1 = cv2.inRange(hsv_planes[0], (0), (25)) 
# cv2.imshow("hue 0-25", mask1)
# mask2 = cv2.inRange(hsv_planes[0], (35), (55))
# cv2.imshow("hue 35-55", mask2)
# mask3 = cv2.inRange(hsv_planes[0], (55), (255)) 
# cv2.imshow("hue 55+", mask3)

cv2.imshow('hsv',hsv)

# bunch = hsv[338:381,266:298,:]
# cv2.imshow('bunch',bunch)
# cv2.waitKey(0)

#waits for user to press any key 
cv2.waitKey(0) 
  
#closing all open windows 
cv2.destroyAllWindows() 