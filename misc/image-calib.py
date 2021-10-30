import cv2
import matplotlib.pyplot as plt
import numpy as np
import sys

def resizeImage(inputImage, scalePercent = 20):
    width = int(inputImage.shape[1] * scalePercent / 100)
    height = int(inputImage.shape[0] * scalePercent / 100)
    dim = (width, height)
    return cv2.resize(inputImage, dim, interpolation = cv2.INTER_AREA)

def getBlackIsolationImageAndMask(inputImage):
    hsv = cv2.cvtColor(inputImage, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0,0,0), (180, 255, 60))
    dst1 = cv2.bitwise_and(inputImage, inputImage, mask=mask)
    return (dst1, mask)

def getGreenCalibrationCircle(inputImage):
    hsv = cv2.cvtColor(inputImage, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (50,50,40), (65, 255, 255)) # H 0-179, S 0-255, V 0-255
    dst1 = cv2.bitwise_and(inputImage, inputImage, mask=mask)
    gray = cv2.cvtColor(dst1, cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.8, 1)
    # ensure at least some circles were found
    if circles is not None:
        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(dst1, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(dst1, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
        # show the output image

    cv2.imshow("output", np.hstack([dst1]))
    cv2.waitKey(0)


def generateFilteredBlackSquareContours(mask):
    widthHeightScaleTolerance = 0.3
    minArea = 1500
    maxArea = 6000

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if not (area > minArea and area < maxArea):
            continue

        rect = cv2.minAreaRect(contour)
        rect = ((rect[0][0], rect[0][1]), (rect[1][0], rect[1][1]), rect[2])
        width = rect[1][0]
        height = rect[1][1]

        box = cv2.boxPoints(rect)
        box = np.int0(box)

        if height < (1 - widthHeightScaleTolerance) * width or height > (1 + widthHeightScaleTolerance) * width:
            continue

        hull = cv2.convexHull(contour)
        precision = 0.15 * cv2.arcLength(hull, True)
        squaredContour = cv2.approxPolyDP(hull, precision, True)
        
        if len(squaredContour) != 4:
            continue

        yield squaredContour






def which(x, values):
    indices = []
    for ii in list(values):
        if ii in x:
            indices.append(list(x).index(ii))
    return indices



mtx = np.array([
    [1.11485379e+03, 0.00000000e+00, 2.11585305e+03],
    [0.00000000e+00, 1.11485379e+03, 2.16951814e+03],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])

dist = np.array([
    [-0.00940794],
    [-0.00084739],
    [-0.00785924],
    [-0.00561968],
    [ 0.00035375],
    [ 0.01409639],
    [ 0.0039594 ],
    [-0.00022776],
    [ 0.        ],
    [ 0.        ],
    [ 0.        ],
    [ 0.        ],
    [ 0.        ],
    [ 0.        ]
])
size_of_marker = 0.02


def processAruco(inputImage, refImage):
    resizedImage = resizeImage(inputImage, scalePercent=40)
    outImage = resizedImage.copy()
    gray = cv2.cvtColor(resizedImage, cv2.COLOR_BGR2GRAY)

    refImage = cv2.cvtColor(refImage, cv2.COLOR_BGR2GRAY)

    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters_create()

    refCorners, refIds, refRejected = cv2.aruco.detectMarkers(refImage, arucoDict, parameters = arucoParams)
    # create bounding box from reference image dimensions
    rect = np.array([[[0,0],
                    [refImage.shape[1],0],
                    [refImage.shape[1],refImage.shape[0]],
                    [0,refImage.shape[0]]]], dtype = "float32")


    (corners, ids, rejected) = cv2.aruco.detectMarkers(resizedImage, arucoDict, parameters=arucoParams)

    # if ids is not None:
    #     idx = which(refIds, ids)
    #     idxOg = which(ids, refIds)
    #     # if any detected marker in frame is also in the reference image
    #     if len(idx) > 0:
    #         refID3 = [refCorners[x] for x in idx]
    #         cornerID3 = [corners[x] for x in idxOg]

    #         cornerTL = cornerID3[0][0][0]
    #         cornerTR = cornerID3[0][0][1]
    #         ogDistance = np.linalg.norm(cornerTL - cornerTR)

    #         refTL = refID3[0][0][0]
    #         refTR = refID3[0][0][1]
    #         refDistance = np.linalg.norm(refTL - refTR)

    #         scale = ogDistance / refDistance

    #         refID3 = np.subtract(refID3, (100, 100)) # Align to (0,0)
    #         refID3 = np.multiply(refID3, scale)
    #         refID3 = np.add(refID3, cornerTL) # Align to point in original image
    #         # refID3 = np.divide(refID3, (3, 3))

    #         # flatten the array of corners in the frame and reference image
    #         these_res_corners = np.concatenate(cornerID3, axis = 1)
    #         these_ref_corners = np.concatenate(refID3, axis = 1)
    #         # these_ref_corners = np.add(these_ref_corners)
    #         # estimate homography matrix
    #         h, s = cv2.findHomography(these_res_corners, these_ref_corners, cv2.RANSAC, 5.0)
    #         # if we want smoothing
    #         this_h = h
    #         # transform the rectangle using the homography matrix
    #         newRect = cv2.perspectiveTransform(rect, this_h, (outImage.shape[1],gray.shape[0]))
    #         # draw the rectangle on the frame
    #         outImage = cv2.polylines(outImage, np.int32(newRect), True, (0,0,0), 10)

    #         height, width, channels = outImage.shape
    #         cv2.aruco.drawDetectedMarkers(outImage,corners,ids)
    #         outImage = cv2.warpPerspective(resizedImage, h, (width, height))
    #     # draw detected markers in frame with their ids

    imaxis = cv2.aruco.drawDetectedMarkers(outImage, corners, ids=ids)
    rvecs,tvecs,_objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, size_of_marker , mtx, dist)
    length_of_axis = 0.02
    for i in range(len(tvecs)):
        # print(i)
        # print(rvecs[i])
        # print(tvecs[i])
        imaxis = cv2.aruco.drawAxis(imaxis, mtx, dist, rvecs[i], tvecs[i], length_of_axis)

    print(_objPoints)

    if len(ids) == 4:
        point1Idx = list(ids).index(4)
        point2Idx = list(ids).index(2)
        point3Idx = list(ids).index(3)
        point4Idx = list(ids).index(1)

        pointRTVecs = (
            (rvecs[point1Idx], tvecs[point1Idx]),
            (rvecs[point2Idx], tvecs[point2Idx]),
            (rvecs[point3Idx], tvecs[point3Idx]),
            (rvecs[point4Idx], tvecs[point4Idx]),
        )

        for i in range(1, 4):
        #    distance = np.linalg.norm(pointRTVecs[i][1] - pointRTVecs[i-1][1])
        #     print(distance * 1000)

        #     R_ref_to_cam = cv2.Rodrigues(pointRTVecs[i-1][1])[0] #reference to camera
        #     R_test_to_cam = cv2.Rodrigues(pointRTVecs[i][1])[0] #test to camera
        #     R_cam_to_ref = np.transpose(R_ref_to_cam) #inverse of reference to camera
        #     R_test_to_ref = np.matmul(R_test_to_cam,R_cam_to_ref) #test to reference
        #     print(R_test_to_ref)
        #     # euler = cv2.decomposeProjectionMatrix()
        #     # print(euler)
        #     # print([math.degrees(i) for i in euler])

            composedRvec, composedTvec = relativePosition(*pointRTVecs[i-1], *pointRTVecs[i])
            R = cv2.Rodrigues(composedRvec)[0]
            print(composedRvec)
            print(rotationMatrixToEulerAngles(R))


    cv2.imshow('frame', np.hstack([resizedImage, outImage]))



import math
def rotationMatrixToEulerAngles(R) :

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([math.degrees(x), math.degrees(y), math.degrees(z)])

def inversePerspective(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    R = np.matrix(R).T
    invTvec = np.dot(-R, np.matrix(tvec))
    invRvec, _ = cv2.Rodrigues(R)
    return invRvec, invTvec


def relativePosition(rvec1, tvec1, rvec2, tvec2):
    rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape(
        (3, 1))
    rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))

    # Inverse the second marker, the right one in the image
    invRvec, invTvec = inversePerspective(rvec2, tvec2)

    orgRvec, orgTvec = inversePerspective(invRvec, invTvec)
    # print("rvec: ", rvec2, "tvec: ", tvec2, "\n and \n", orgRvec, orgTvec)

    info = cv2.composeRT(rvec1, tvec1, invRvec, invTvec)
    composedRvec, composedTvec = info[0], info[1]

    composedRvec = composedRvec.reshape((3, 1))
    composedTvec = composedTvec.reshape((3, 1))
    return composedRvec, composedTvec



refimgpath = "C:\\Users\\chris\\Downloads\\4x4_1000-3.png"
refImage = cv2.imread(refimgpath)
refImage = cv2.copyMakeBorder(refImage, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=(255, 255, 255))
refImage = cv2.rotate(refImage, cv2.ROTATE_180)




# stream = cv2.VideoCapture("http://192.168.0.107:8080/video")
# stream.set(cv2.CAP_PROP_BUFFERSIZE, 2)

# import cv2.aruco as aruco


# if not stream.isOpened():
#     print ("Cannot open stream")
#     sys.exit()

# while True:
#     ret, frame = stream.read()
#     if not ret:
#         print ("Can't receive frame")
#         break
    
#     processAruco(frame, refImage)

#     if cv2.waitKey(1) == ord('q'):
#         break

# stream.release()
# cv2.destroyAllWindows()

# sys.exit()






imgpath = "C:\\Users\\chris\\Downloads\\PXL_20210914_035459168.jpg"
originalImage = cv2.imread(imgpath)

processAruco(originalImage, refImage)

cv2.waitKey(0)

cv2.destroyAllWindows()

sys.exit()





resizedImage = resizeImage(originalImage)
bluredImage = cv2.GaussianBlur(resizedImage, (5,5), 0)

getGreenCalibrationCircle(bluredImage)

# cv2.imshow("output", np.hstack([resizedImage, greenImage]))
# cv2.waitKey(0)

blackImage, blackMask = getBlackIsolationImageAndMask(bluredImage)

for contour in generateFilteredBlackSquareContours(blackMask):
    # cv2.drawContours(blackImage,[contour], 0, (0, 0, 255), 2)
    # cv2.drawContours(blackImage,[hull], 0, (0, 255, 0), 2)
    print(contour)
    cv2.drawContours(resizedImage,[contour], 0, (0, 0, 255), 1)
    cv2.circle(blackImage, contour[0][0], 2, (255, 255, 0), 1)
    cv2.circle(blackImage, contour[1][0], 2, (0, 255, 0), 1)
    cv2.circle(blackImage, contour[2][0], 2, (0, 0, 255), 1)
    cv2.circle(blackImage, contour[3][0], 2, (0, 255, 255), 1)


# calibContour = list(generateFilteredBlackSquareContours(blackMask))[0]
# rect = np.float32((
#     (calibContour[0][0][0] + 2, calibContour[0][0][1] + 2),
#     (calibContour[1][0][0], calibContour[1][0][1]),
#     (calibContour[2][0][0], calibContour[2][0][1]),
#     (calibContour[3][0][0], calibContour[3][0][1]),
# ))
# print(rect)
# homography, mask = cv2.findHomography(calibContour, rect, cv2.RANSAC, 10)
# warpedImage = cv2.warpPerspective(resizedImage, homography, (resizedImage.shape[1], resizedImage.shape[0]))


cv2.imshow("output", np.hstack([resizedImage, blackImage]))
cv2.waitKey(0)
