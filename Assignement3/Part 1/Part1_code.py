import cv2
import numpy as np
import matplotlib.pyplot as plt


#function

def threshold(Tmin, Tmax, image):
    # grab the image dimensions
    h = image.shape[0]
    w = image.shape[1]
    
    # loop over the image, pixel by pixel
    for y in range(0, h):
        for x in range(0, w):
            # threshold the pixel
            if (image[y,x] >= Tmin) and (image[y,x] <= Tmax):
            	image[y, x] = 255
            else:
             	image[y, x] = 0

            
    # return the thresholded image
    return image


        

#------------ Open Image ------------
#path of the picutre
name = "img/coins.png"

img = cv2.imread(name)                              #open the image
img_gray = cv2.imread(name,cv2.IMREAD_GRAYSCALE)    #open the image in grey

cv2.imshow('grey', img_gray)                        #show the image
cv2.imwrite("report_pic/img_gray.png", img_gray)    #save the image
cv2.waitKey(0)       #wait for a key to exit

#------------ HSV ------------
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)          #transform the image in HSV


h, s, v = cv2.split(hsv)        #split HSV picture
cv2.imshow("Hue", h)
cv2.imshow("Ssaturation", s)
cv2.imshow("Value", v)


cv2.imwrite("report_pic/hsv/h.png", h)
cv2.imwrite("report_pic/hsv/s.png", s)
cv2.imwrite("report_pic/hsv/v.png", v)
cv2.waitKey(0)

#------------ Binary ------------
for_binary = s              #keep the saturation one for the binary

# find frequency of pixels in range 0-255
hist_min = 0
hist_max = 255
histr = cv2.calcHist([for_binary],[hist_min],None,[hist_max],[hist_min,hist_max])

binary_img = threshold(75,255, for_binary)  #keep values from 75 to 255

#plot Histogram
plt.plot(histr)                             
plt.show()

cv2.imshow('binary imgage',binary_img)
cv2.imwrite("report_pic/binary_img.png", binary_img)

cv2.waitKey(0)


#------------ Dilation and Erotion ------------
kernel_dilation_erosion = np.ones((3,3),np.uint8)   #Create a Kernel for erosion and dilation
iteration = 1                                       #chose the number of itaration 

dilation = cv2.dilate(binary_img,kernel_dilation_erosion,iterations = iteration)
erosion = cv2.erode(dilation,kernel_dilation_erosion    ,iterations = iteration)

cv2.imshow('Dilation', dilation)
cv2.imshow('Erosion', erosion)

cv2.imwrite("report_pic/erosion_dilation/dilation.png", dilation)
cv2.imwrite("report_pic/erosion_dilation/erosion.png", erosion)
cv2.waitKey(0)

#------------ Gaussian Filter ------------
kernel_gaussian = (5,5)                             #Create a Kernel for gaussian filter
blur_img = cv2.GaussianBlur(erosion,kernel_gaussian,cv2.BORDER_DEFAULT)

cv2.imshow("Gaussian Smoothing",blur_img)
cv2.imwrite("report_pic/Gaussian_Smoothing.png", blur_img)
cv2.waitKey(0)

#------------ Edge Detection ------------
edge_detection = cv2.Canny(blur_img, 10, 250)       
cv2.imshow("Edge_detection", edge_detection)
cv2.imwrite("report_pic/Edge_detection.png", edge_detection)
cv2.waitKey(0)

