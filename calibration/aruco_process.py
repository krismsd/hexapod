import statistics
import cv2
import numpy as np
import math

from joint import COXA, FEMUR, TIBIA


class ArucoProcessor:
    lineColor = (255, 255, 255)
    textColor = (0, 0, 0)
    circle1Color = (255, 0, 0)
    circle2Color = (0, 255, 0)

    def __init__(self):
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.arucoParams.cornerRefinementWinSize = 3

    def processAruco(self, inputImage, runningCalibration, calibrationCrossActive):
        outImage = inputImage.copy()
        # outImage = cv2.cvtColor(inputImage, cv2.COLOR_BGR2GRAY)
        # outImage = cv2.cvtColor(outImage,cv2.COLOR_GRAY2BGR)

        (corners, ids, rejected) = cv2.aruco.detectMarkers(inputImage, self.arucoDict, parameters=self.arucoParams)
        cv2.aruco.drawDetectedMarkers(outImage, corners, ids=ids)

        legMarkerIds = (0,1,2,3,4,5,6) # ids ordered as (*side(ref, tibia, femur, end), *top(ref, coxa, end))
        markerCenters = []
        for id in legMarkerIds:
            if ids is None or id not in ids:
                markerCenters.append(None)
                continue

            idx = list(ids).index(id)
            calcOrdMean = lambda ord: int(statistics.mean(corners[idx][0][i][ord] for i in range(4)))
            markerCenters.append(tuple((
                calcOrdMean(0), # x
                calcOrdMean(1), # y
            )))


        self.processJoint(markerCenters[0:3], outImage, FEMUR, runningCalibration, calibrationCrossActive)
        self.processJoint(markerCenters[1:4], outImage, TIBIA, runningCalibration, calibrationCrossActive)
        self.processJoint(markerCenters[4:7], outImage, COXA, runningCalibration, calibrationCrossActive)


        if calibrationCrossActive:
            midPoint = np.array((int(outImage.shape[1] / 2), int(outImage.shape[0] / 2)))
            crossColor = (0,0,0)
            calibrateTargetPoints = (markerCenters[1], markerCenters[2], markerCenters[5])
            if any(p is not None and pointChainSize((midPoint, p)) < 10 for p in calibrateTargetPoints):
                crossColor = (255,255,255)

            cv2.line(outImage, midPoint-(100,100), midPoint+(100, 100), crossColor, 2)
            cv2.line(outImage, midPoint+(-100,100), midPoint+(100, -100), crossColor, 2)

        lines = []
        if runningCalibration is not None:
            lines.append("Calibration: {}".format(runningCalibration.joint))
            lines.append("State: {}".format(runningCalibration.state))
            lines.append("ms: {}".format(runningCalibration.lastMs))
            lines.append("angle limit: {} - {}".format(runningCalibration.startAngle, runningCalibration.endAngle))
            lines.append("current angle: {} - {} samples".format(runningCalibration.currentAngle, len(runningCalibration.samplesOfCurrentAngle)))
            
            lines.append("")

            lines.append("Time elapsed: {}".format(timeStringFromSeconds(runningCalibration.getTimeElapsed())))
            lines.append("Time remaining: ~{}".format(timeStringFromSeconds(runningCalibration.getTimeRemaining())))

            lines.append("")
            lines.append("Samples for current angle:")
            for sample in runningCalibration.samplesOfCurrentAngle:
                lines.append("{}ms - {:+}deg".format(*sample))

        else:
            lines.append("Calibration: not running")

        offset = 20
        for i in range(len(lines)):
            drawTextWithBg(outImage, lines[i], font=cv2.FONT_HERSHEY_SIMPLEX, pos=(5, i*offset+5), font_scale=0.5, text_color=self.textColor, font_thickness=2)

        
        cv2.imshow('frame', outImage)
        # cv2.imshow('frame', np.hstack([inputImage, outImage]))


    def processJoint(self, points, outImage, joint, runningCalibration, calibrationCrossActive):
        if isAnyNone(points):
            return None

        baselinePoint, originPoint, movingPoint = points

        if calibrationCrossActive:
            # Draw origin -> moving guiding circle
            distance = pointChainSize((originPoint, movingPoint))
            cv2.circle(outImage, originPoint, distance, self.circle1Color, 2)

        cv2.line(outImage, baselinePoint, originPoint, self.lineColor, 1)
        cv2.line(outImage, originPoint, movingPoint, self.lineColor, 1)

        jointAngle = degreesBetweenPointVectors(originPoint, baselinePoint, movingPoint)

        if runningCalibration and runningCalibration.joint == joint:
            runningCalibration.pushSample(jointAngle)

            # Add calibration decoration
            for angle in range(runningCalibration.startAngle, runningCalibration.endAngle + 1, runningCalibration.stepAngle):
                endPoint = rotatePointFromOrigin(originPoint, baselinePoint, math.radians(angle))
                color = (0, 255, 0) if angle in runningCalibration.angleMapping else \
                        (255, 0, 0) if runningCalibration.currentAngle == angle else \
                        (0, 0, 255)
                cv2.line(outImage, originPoint, endPoint, color, 1)

        drawTextWithBg(outImage, str(jointAngle), font=cv2.FONT_HERSHEY_SIMPLEX, pos=originPoint, font_scale=1, text_color=self.textColor, font_thickness=2)


def isAnyNone(iter):
    return any(x is None for x in iter)


def degreesBetweenPointVectors(origin, xy1, xy2, roundDigits=2):
    # Taken from https://stackoverflow.com/a/58956483
    x1, y1 = xy1
    x2, y2 = origin
    x3, y3 = xy2

    deg1 = (360 + math.degrees(math.atan2(x1 - x2, y1 - y2))) % 360
    deg2 = (360 + math.degrees(math.atan2(x3 - x2, y3 - y2))) % 360
    
    angle = deg2 - deg1 if deg1 <= deg2 else 360 - (deg1 - deg2)
    return round(angle, roundDigits)


def rotatePointFromOrigin(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    angle = (2 * math.pi) - angle

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    # qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    # qy = oy - math.sin(angle) * (px - ox) - math.cos(angle) * (py - oy)
    return (int(qx), int(qy))


def pointChainSize(points):
    size = 0
    prevPoint = points[0]
    for point in points[1:]:
        size += int(math.sqrt(((prevPoint[0]-point[0])**2)+((prevPoint[1]-point[1])**2)))
        prevPoint = point
    return size


def drawTextWithBg(img, text,
        font=cv2.FONT_HERSHEY_PLAIN,
        pos=(0, 0),
        font_scale=3,
        font_thickness=2,
        text_color=(0, 255, 0),
        text_color_bg=(0, 0, 0)):
    x, y = pos
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size

    sub_img = img[y-5:y+text_h+5, x-5:x+text_w+5]
    white_rect = np.ones(sub_img.shape, dtype=np.uint8) * 255

    res = cv2.addWeighted(sub_img, 0.5, white_rect, 0.5, 1.0)
    img[y-5:y+text_h+5, x-5:x+text_w+5] = res

    cv2.putText(img, text, (x, int(y + text_h + font_scale - 1)), font, font_scale, text_color, font_thickness)

def timeStringFromSeconds(secs):
    hours, rem = divmod(secs, 3600)
    mins, secs = divmod(rem, 60)
    return "{:0>2}:{:0>2}:{:0>2}".format(hours, mins, secs)
