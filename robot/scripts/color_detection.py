import numpy as np
import cv2

def detectColor(image):
    #image = cv2.imread("frame0008.jpg")

    #crop image as the ring will only be seen in upper half of camera image
    image = image[0:240, 0:690]

    #boundary for how many nonZero pixels have to be found to consider color detected
    detectBoundary = 800

    #boundaries for red, blue, green respectively
    boundaries = [
        ([0, 100, 100], [12, 255, 255]),
        ([110, 100, 100], [130, 255, 255]),
        ([36, 50, 50], [86, 255, 255])
    ]

    colors = ["red", "blue", "green"]

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    i = 0
    for (lower, upper) in boundaries:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")
    
        mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(image, image, mask = mask)
        countNonZero = np.count_nonzero(output)
        #print(countNonZero)

        #color detected
        if countNonZero > detectBoundary:
            #print("Found {}".format(colors[i])) 
            return i
            #break

        #cv2.imshow("images", np.hstack([image, output]))
        #cv2.waitKey(0)
        i += 1
    return i