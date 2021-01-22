import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3

import cv2
#sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy
import numpy as np
import glob

PERCENTAGE = 0.35  # indicate how an image should be resized into


# this function downscalles every image in the list of images given. This
# images are downsized by the percentage amount
class ObstacleDetection:

    def downscale(img, percent):
        width = int(img.shape[1] * percent)
        height = int(img.shape[0] * percent)
        dim = (width, height)

        resized = cv2.resize(img, dim)

        return resized


    # This function resizes all the images inthe given list by a PERCENTAGE
    def resizeAll(images):
        retImages = []
        for img in images:
            retImages.append(downscale(img, percent=PERCENTAGE))
        return retImages


    # the function will read all the images into a list from the simutosot file
    def readImg():
        images = np.array([cv2.imread(file) for file in glob.glob("simurosot/*.jpg")])
        return images


    # Perform the closing morphological operations in alist of images
    # and return the new images in alist

    def getClosing(images):
        res = []
        for img in images:
            kernel = np.ones((2, 2), np.uint8)
            closing = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
            res.append(closing)
        return res


    # Perform the opeining morphological operation on the list of images passed into
    # function

    def getOpening(images):
        res = []
        for img in images:
            kernel = np.ones((2, 2), np.uint8)
            opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
            res.append(opening)
        return res

    def getOPeningSingleImage(self,image):
        kernel = np.ones((2, 2), np.uint8)
        opening = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        return opening

    # Draw a boundary rectangle
    def boundingRectangle(self,x, y, w, h, img):
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)


    # draw a bounding rectangle for a list of images that are passed int o the function

    def boundingRectangleAllImages(contourValues, img):
        for contour, img1 in zip(contourValues, img):
            x, y, w, h = contour
            boundingRectangle(x, y, w, h, img1)


    # Purpose: Get the mask of the image given the lower and upper number and return the masked images
    # return : The function should return the list of masked images in the file
    def getMask(images, lower, upper):
        mask_images = []
        for img in images:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)
            mask_images.append(mask)
        return mask_images


    def getMaskSingleImage(self ,image, lower, upper):
        # The function gets the mask a singke image 
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        return mask


    # Purpose : Get the contour value of an image and draw a rectangle box on the contour area given a contour threshold
    # Retun : The function should return the x,y,w,h values if the object
    def getContours(self,img, img_copy):
        # we find the contours of an image
        # recicev  the outer corners(by request for all contours
        #contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        _,contours,_  = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # loop through the contours
        x = 0
        y = 0
        w = 0
        h = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # print(area)

            # give minimum threshold so that we dont detetct any noise (area of 500)
            if area > 500:

                peri = cv2.arcLength(cnt, True)  # the contour parameter
                #print("Area is {}".format(area))
                approx = cv2.approxPolyDP(cnt, 0.01 * peri, True)  # should give the coner points of shapes

                # using a rectanglar box
                x, y, w, h = cv2.boundingRect(approx)

            cv2.rectangle(img_copy, (x, y), (x + w, y + h), (0, 255, 0), 2)
               # Showing the final image. 
        cv2.imshow('image2', img_copy)  

        # # Exiting the window if 'q' is pressed on the keyboard. 
        if cv2.waitKey(0) & 0xFF == ord('q'):  
                cv2.destroyAllWindows()
        return x, y, w, h




    # This function will sort the item by the provided key 
    def getXfromRect(self,item):
        return item[0]



    """ 
    Function: get_contour_coordinates 
    param :
    The fucnction retuns a list of all the x,y,w,h cordinates of bounding box on each obstacle after merging any small
        boexs within the main box 

    """
    def get_contour_coordinates(self , img , img2 ) :
        # ( black and white only image). 
        font = cv2.FONT_HERSHEY_COMPLEX 
        #_, threshold = cv2.threshold(img, 110, 255, cv2.THRESH_BINARY) 
  
        allCOrds = []
        rect= []
        rectsUsed = [] 
        _,contours,_  = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # Going through every contours found in the image. 
        x = 0
        y = 0
        w = 0
        h = 0
        for cnt in contours :
            area = cv2.contourArea(cnt)

            if area> 500:

  
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.01 * peri, True)
              
                x, y, w, h = cv2.boundingRect(approx)
                allCOrds.append((x,y,w,h))

                rect.append(cv2.boundingRect(approx))
                rectsUsed.append(False)

                # sorting rectangles by the x-axis
                rect.sort(key = self.getXfromRect)
                #merge the rectangles by similar x-axis 
                self.mergeBoxes(rect,rectsUsed,img2)
                cv2.rectangle(img2, (x, y), (x + w, y + h), (0, 255, 0), 2)
                stringx = str(x) +" " + str(y)

                cv2.putText(img2,stringx, (x,y),font,0.5,(0,255,0))
  
        # Showing the final image. 
        #cv2.imshow('image2', img2)  
  
        # # Exiting the window if 'q' is pressed on the keyboard. 
        #if cv2.waitKey(0) & 0xFF == ord('q'):  
         #  cv2.destroyAllWindows() 
        return allCOrds


    def mergeBoxes(self,rects, rectsUsed,image ):

        xThr = 500
        acceptedRects = []
        for idx, val in enumerate(rects):
            if (rectsUsed[idx]== False):
                  # Initialize current rect
                currxMin = val[0]
                currxMax = val[0] + val[2]
                curryMin = val[1]
                curryMax = val[1] + val[3]

                # This bounding rect is used
                rectsUsed[idx] = True


                # Iterate all initial bounding rects
                # starting from the next
                for subIdx, subVal in enumerate(rects[(idx+1):], start = (idx+1)):

                    # Initialize merge candidate
                    candxMin = subVal[0]
                    candxMax = subVal[0] + subVal[2]
                    candyMin = subVal[1]
                    candyMax = subVal[1] + subVal[3]

                    # Check if x distance between current rect
                    # and merge candidate is small enough
                    if (candxMin <= currxMax + xThr):

                        # Reset coordinates of current rect
                        currxMax = candxMax
                        curryMin = min(curryMin, candyMin)
                        curryMax = max(curryMax, candyMax)

                        # Merge candidate (bounding rect) is used
                        rectsUsed[subIdx] = True
                    else:
                        break

                # No more merge candidates possible, accept current rect
                acceptedRects.append([currxMin, curryMin, currxMax - currxMin, curryMax - curryMin])

        for rect in acceptedRects:
            img = cv2.rectangle(image, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (121, 11, 189), 2)
        #cv2.imshow('img3',image)
        # Exiting the window if 'q' is pressed on the keyboard. 
        # if cv2.waitKey(0) & 0xFF == ord('q'):  
        #     cv2.destroyAllWindows() 






    # This function get the contour value of the list of images passed into the function
    # retun: The functiuon should return the list of the conotur value of all theimagbes in the list
    def getAllContour(images, originalImage):
        # get the contour images of all the images in the list pf images
        contourImages = []
        for img, imgCopy in zip(images, originalImage):
            contourImages.append(getContours(img, imgCopy))
        return contourImages


    # The function should add the mask of all the images in the list

    def addMask(mask1, mask2):
        valList = []
        for ma1, ma2 in zip(mask1, mask2):
            val = ma1 + ma2
            valList.append(val)
        return valList


    """
    Function: getObstacleSingleImage 
    param
    The function does obstacle detection on a single

    """
    def getObstacleSingleImage(self, image):
        # This function will get the box obstacle in the image 
        red = np.array([121, 161, 0, 179, 255, 255])
        white = np.array([116, 0, 0, 179, 255, 175])
        lower_white = np.array(white[0:3])
        upper_white = np.array(white[3:6])

        lower = np.array(red[0:3])
        upper = np.array(red[3:6])

        # get the mask 
        mask1 = self.getMaskSingleImage(image, lower,upper)
        mask2 = self.getMaskSingleImage(image,lower_white,upper_white)
        image_copy = image.copy()

        new_open = self.getOPeningSingleImage(mask2)
        final = mask1 + new_open

        
        #x,y,w,h = self.get_contour_coordinates(final,image_copy)
        list1 = self.get_contour_coordinates(final,image_copy)
        return list1
        #self.get_contour_coordinates(final,image_copy)
        #self.boundingRectangle(x,y,w,h,image_copy)
        #cv2.imwrite("Obstacle.jpg", image_copy)






# read in the image from the rvobti 


# while True:
#     frame = resizeAll(readImg())

#     red = np.array([121, 161, 0, 179, 255, 255])
#     white = np.array([116, 0, 0, 179, 255, 175])
#     # this gives us the result bafter from nthe track pads movement
#     # define the lower bound and the upper bound
#     lower_white = np.array(white[0:3])
#     upper_white = np.array(white[3:6])

#     lower = np.array(red[0:3])
#     upper = np.array(red[3:6])

#     mask = getMask(frame, lower, upper)

#     mask2 = getMask(frame, lower_white, upper_white)
#     opening_list = getOpening(mask2)
#     cllosing_list = getClosing(opening_list)

#     img_copy = frame.copy()
#     finalImage = addMask(mask, opening_list)

#     getAllContour(finalImage, img_copy)

#     smImage = np.hstack(img_copy[0:(len(img_copy) // 2)])
#     sm2 = np.hstack(img_copy[-(len(img_copy) // 2):])
#     ver = np.vstack((sm2, smImage))
#     cv2.imshow("<Mask>", ver)

#     cv2.waitKey(1)
#     key = cv2.waitKey(1)
#     if key == ord('q'):
#         break;

# cv2.destroyWindow()
