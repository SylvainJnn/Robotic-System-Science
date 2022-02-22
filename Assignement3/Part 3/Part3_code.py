import cv2
import numpy as np
import matplotlib.pyplot as plt
import math

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

def get_pixels_width(img):
    max = 0
    min = 10000
    for y in range(len(img)):
        for x in range(len(img[y])):
            if(img[y,x] != 0):
                #check min
                if(x < min):
                    min = x
                #check max
                elif(x > max):
                    max = x
    return(min, max)
    
def calculate_distance(img, h, fov):
    min, max = get_pixels_width(img)
    dp = max-min
    width = img.shape[1]

    distance = width/dp * h/(2 * np.tan(fov/2))
    return(distance)


if(__name__ == '__main__'):
    #------------ Open Image ------------
    #path of the picutre
    name = "pic/part3_pic.jpg"

    img = cv2.imread(name)                              #open the image

    scale_percent = 20 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    cv2.imshow('resized', resized)    
    cv2.waitKey(0)       #wait for a key to exit



    #------------ BRG ------------
    b, r, g = cv2.split(resized)

    cv2.imshow('Blue', b)
    cv2.imshow('Red', r)
    cv2.imshow('Green', g)
    cv2.waitKey(0)

    cv2.imwrite("report_pic/brg/b.png", b)
    cv2.imwrite("report_pic/brg/r.png", r)
    cv2.imwrite("report_pic/brg/g.png", g)

    #------------ HSV ------------
    hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)          #transform the image in HSV
    h, s, v = cv2.split(hsv)                                #split HSV picture
    cv2.imshow("Hue", h)
    cv2.imshow("Saturation", s)
    cv2.imshow("Value", v)

    cv2.imwrite("report_pic/hsv/h.png", h)
    cv2.imwrite("report_pic/hsv/s.png", s)
    cv2.imwrite("report_pic/hsv/v.png", v)
    cv2.waitKey(0)


    #------------ Binary ------------
    for_binary = s           #keep the saturation one for the binary

    # find frequency of pixels in range 0-255
    hist_min = 0
    hist_max = 255
    histr = cv2.calcHist([for_binary],[hist_min],None,[hist_max],[hist_min,hist_max])

    binary_img = threshold(0,164, for_binary)  

    #plot Histogram
    plt.plot(histr)                             
    plt.show()

    cv2.imshow('binary imgage',binary_img)
    cv2.imwrite("report_pic/binary_img.png", binary_img)

    cv2.waitKey(0)


    #------------ Dilation and Erotion ------------
    kernel_dilation_erosion = np.ones((5,5),np.uint8)   #Create a Kernel for erosion and dilation
    iteration = 2                                       #chose the number of itaration 

    dilation = cv2.dilate(binary_img,kernel_dilation_erosion,iterations = iteration)
    erosion = cv2.erode(dilation,kernel_dilation_erosion    ,iterations = iteration+2)

    cv2.imshow('Dilation', dilation)
    cv2.imshow('Erosion', erosion)

    cv2.imwrite("report_pic/dilation.png", dilation)
    cv2.imwrite("report_pic/erosion.png", erosion)
    cv2.waitKey(0)


    #------------ Gaussian Filter ------------
    kernel_gaussian = (5,5)                             #Create a Kernel for gaussian filter
    blur_img = cv2.GaussianBlur(erosion,kernel_gaussian, cv2.BORDER_DEFAULT)

    cv2.imshow("Gaussian Smoothing",blur_img)
    cv2.imwrite("report_pic/Gaussian_Smoothing.png", blur_img)
    cv2.waitKey(0)

    #------------ Edge Detection ------------
    edge_detection = cv2.Canny(blur_img, 10, 250)       
    cv2.imshow("Edge_detection", edge_detection)
    cv2.imwrite("report_pic/Edge_detection.png", edge_detection)
    cv2.waitKey(0)


    #------------ get pixel ------------

    fov = 64
    object_size = 0.045

    distance = calculate_distance(edge_detection, object_size, math.radians(fov))
    print("the distance between the camera and the object is : ", distance, "m")
