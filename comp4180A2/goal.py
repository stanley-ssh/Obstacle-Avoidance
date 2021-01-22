import math

import cv2
import numpy as np
import glob

print("package Imported!")

PERCENTAGE = 0.35


# The function will resize all the images in the list of all teh images
class Goal:

    def resizeAll(images):
        retImages = []
        for img in images:
            retImages.append(downscale(img, percent=PERCENTAGE))
        return retImages


    # This function will downsacle the image by a percenatage(PERCENTAGE )
    # The function will return the list of all images after resizeing

    def downscale(img, percent):
        width = int(img.shape[1] * percent)
        height = int(img.shape[0] * percent)
        dim = (width, height)

        resized = cv2.resize(img, dim)

        return resized


    # The function will read all the image sin the file and return a listof all the images
    def readImg():
        images = np.array([cv2.imread(file) for file in glob.glob("simurosot/*.jpg")])
        return images


    # Purpose : the function takes alist of all the upper and lower HUE values and get s the mask of all the images
    # The fuction will return the  mask images of all the passed in images
    def getMask(images, lower, upper):
        mask_images = []
        for img in images:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # lower = np.array([h_min, s_min, v_min])
            # upper = np.array([h_max, s_max, v_max])
            mask = cv2.inRange(hsv, lower, upper)
            mask_images.append(mask)
        return mask_images


    # Draw a bounding rectangle along the passed in image
    def boundingRectangle(self, x, y, w, h, img):
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

    #Get the boundig box of all the [assed in images given the countour values
    def boundingRectangleAllImages(contourValues, img):
        for contour, img1 in zip(contourValues, img):
            x, y, w, h = contour
            # if (x,y,w,h == []):
            #     print(x,y,w,h)

            boundingRectangle(x, y, w, h, img1)


    # the function return the position of the bounding rectangle when a contour is found
    def getContours(self,img, img_copy):
        # we find the contours of an image
        # recicev  the outer corners(by request for all contours
        _,contours,_  = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # loop through the contours
        x = 0
        y = 0
        w = 0
        h = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # print(area)

            # give minimum threshold so that we dont detetct any noise (area of 5)
            if area > 10:
                cv2.drawContours(img_copy, cnt, -1, (255, 0, 0), 3)
                peri = cv2.arcLength(cnt, True)  # the contour parameter
                # print("Area is {}".format(area))
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)  # should give the coner points of shapes

                # using a rectanglar box
                x, y, w, h = cv2.boundingRect(approx)

                # cv2.rectangle(img_copy, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # adding text to the identified objects
        return x, y, w, h





    def getAllContour(images, originalImage):
        # get the contour images of all the images in the list pf images
        contourImages = []
        for img, imgCopy in zip(images, originalImage):
            contourImages.append(getContours(img, imgCopy))
        return contourImages


    # This function returns the lines in the image and also draws the line in the image
    def FLD(self,image, diff):
        # Create default Fast Line Detector class

        fld = cv2.ximgproc.createFastLineDetector(20, 1.4114, 50, 50, 3, True)

        # Get line vectors from the image
        lines = fld.detect(image)
        # Draw lines on the image
        line2 = lines[0][0]
        line_on_image = fld.drawSegments(diff, lines)
        # Plot
        cv2.imshow("LineOnImage", line_on_image)
        return lines


    def getClosing(images):
        res = []
        for img in images:
            kernel = np.ones((2, 2), np.uint8)
            closing = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
            res.append(closing)
        return res


    def getAllFLD(images):
        res = []
        for img in (images):
            res.append(FLD(img))
        return res


    def lineDetection(lower, upper, images):
        img_copy = images.copy()
        mask = getMask(images, lower, upper)
        closing = getClosing(mask)
        fld = getAllFLD(closing)
        return fld


    def drawLines(fld, frame):
        for lines in fld:
            # (lines)
            cv2.line(frame, (lines[0], lines[1]), (lines[2], lines[3]), (255, 0, 0), 2)


    def detect(self,fld, images):
        for fd, img in zip(fld, images):
            drawLines(fd, img)


    def getLine(lower, upper):
        frame = resizeAll(readImg())
        img_copy = frame.copy()
        det = lineDetection(lower, upper, frame)
        print(det)

        lines = []
        for line in det:
            lines.append(line)
            new_line = []
            for lin in line:
                new_line.append(lin)
            detect(new_line, img_copy)

        smImage = np.hstack(img_copy[0:(len(img_copy) // 2)])
        sm2 = np.hstack(img_copy[-(len(img_copy) // 2):])
        ver = np.vstack((sm2, smImage))
        cv2.imshow("<Mask>", ver)


    # find the goal given one single image ( this function will find the blue goal post )
    # the function should retuen the points of the goal 
    def find_goal_single_image (self , image):
        lower = np.array([82, 219, 187])
        upper = np.array([142, 255, 255])
        hsv = cv2.cvtColor(image , cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        image_copy = image.copy()
        x, y, w, h  = self.getContours(mask,image_copy)
        #self.boundingRectangle(x,y,w,h,image_copy)
        return x,y,w,h
        


    def findGoal(lower, upper):
        frame = resizeAll(readImg())
        mask = getMask(frame, lower, upper)
        img_copy = frame.copy()
        contouValues = getAllContour(mask, img_copy)
        boundingRectangleAllImages(contouValues, img_copy)
        smImage = np.hstack(img_copy[0:(len(img_copy) // 2)])
        sm2 = np.hstack(img_copy[-(len(img_copy) // 2):])
        ver = np.vstack((sm2, smImage))
        cv2.imshow("<Mask>", ver)


    def dist(self,line1, line2):
        a = (line1[3] - line1[1])
        b = line1[2] - line1[0]
        c1 = (line1[0] * line1[3]) - (line1[1] * line1[2])
        c2 = (line2[0] * line2[3]) - (line2[1] * line2[2])
        dist = abs(c2 - c1) / math.sqrt(a * a + b * b)
        return dist


    def getDist(self,lines):
        distances = []
        for line in lines:
            x, y, x1, y1 = line[0], line[1], line[2], line[3]

            for li in lines:
                x2, y2, x3, y3 = li[0], li[1], li[2], li[3]
                if x != x1:
                    dist1 =self. dist(line, li)
                    distances.append((line, li, dist1))
        return distances


    def Q4(self,frame):
        #frame = cv2.imread("simurosot/default_gzclient_camera(1)-2020-09-15T10_12_58.242557.jpg")
        # frame = downscale(frame,PERCENTAGE)
        # frame = resizeAll(readImg())
        img_copy = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # convert thge image into HSV
        lower_line = np.array([0, 71, 0])
        upper_line = np.array([160, 255, 193])

        mask = cv2.inRange(hsv, lower_line, upper_line)

        kernel = np.ones((2, 2), np.uint8)
        height, width = mask.shape
        closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # closing = cv2.bitwise_not(closing)

        # closing = closing + mask_red + mask_goal
        fld = self.FLD(closing, img_copy)
        # print(fld)
        # print()
        # print(fld[0])
        lines = []

        for cl in fld:
            lines.append(cl[0])

        # print(parra)
        dio = self.getDist(lines)
        lio = []
        lio2 = []
        for a in dio:
            l1, l2, di = a

            if di == 0:
                new_li = [l1[0], l1[1], l2[2], l2[3]]
                lio.append(l1)
            elif di > 300:
                new_li = [l1[0], l1[1], l2[2], l2[3]]
                lio2.append(new_li)

        for lines in lines:
            # (lines)
            cv2.line(frame, (lines[0], lines[1]), (lines[2], lines[3]), (255, 0, 0), 1)
            cv2.imshow("Original", frame)
            cv2.imwrite("line2.jpg", frame)




# while True:
#     lower = np.array([82, 219, 187])
#     upper = np.array([142, 255, 255])

#     gol = Goal()
#     frame = cv2.imread("image.jpg")
#     dat = gol.find_goal_single_image(image = frame)
#     cv2.imshow("detected", dat )
#     #findGoal(lower, upper)
#     #Q4()
#     key = cv2.waitKey(1)
#     if key == ord('q'):
#         break;

    # cv2.destroyWindow()
