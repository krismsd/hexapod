import math

# All lengths are only on x-axis assuming tibia + femur are lying flat y-axis (in their own perspective)
# It is assumed that leg is built so tibia is in line with coxa motion point (y differences cancel out)
coxaLength = 32.10 # Distance of coxa-body motion point to coxa-femur motion point
femurLength = 70 # Distance of femur-coxa motion point to femur-tibia motion point
tibiaLength = 100.12 # Distance of tibia-femur motion point to end of tibia

calibrationTimings = {
    "C0": [(150, 1758.8), (152, 1742.2), (154, 1714), (156, 1698.2), (158, 1669.8), (160, 1644.4), (162, 1621), (164, 1600.2), (166, 1572.8), (168, 1550.6), (170, 1520.6), (172, 1497.8), (174, 1471), (176, 1449.8), (178, 1423.8), (180, 1399.8), (182, 1374.2), (184, 1350.8), (186, 1332.2), (188, 1305), (190, 1281.6), (192, 1263.8), (194, 1237.6), (196, 1216), (198, 1187), (200, 1164.2), (202, 1144.4), (204, 1117.8), (206, 1094), (208, 1074.2), (210, 1049.4)],
    "F0": [(90, 636.2), (92, 656.8), (94, 678.6), (96, 704.6), (98, 725.2), (100, 741.8), (102, 767.2), (104, 798.8), (106, 818.4), (108, 830), (110, 855.6), (112, 872.8), (114, 900.8), (116, 922.4), (118, 939.8), (120, 966), (122, 985.8), (124, 1008.2), (126, 1020.6), (128, 1052.6), (130, 1073.8), (132, 1099.4), (134, 1118.4), (136, 1135.8), (138, 1157.4), (140, 1181), (142, 1209.2), (144, 1228.8), (146, 1257.4), (148, 1283), (150, 1306.2), (152, 1327), (154, 1354), (156, 1366.6), (158, 1392.4), (160, 1424.2), (162, 1446.2), (164, 1468.6), (166, 1489.2), (168, 1514.6), (170, 1534.6), (172, 1558.4), (174, 1580.8), (176, 1601.4), (178, 1621.6), (180, 1644.2), (182, 1670.4), (184, 1689.8), (186, 1713.6), (188, 1729.8), (190, 1757.4), (192, 1777.6), (194, 1794.8), (196, 1818.2), (198, 1841.6), (200, 1867.6), (202, 1888.6), (204, 1905.8), (206, 1927.2), (208, 1947.6), (210, 1965.8), (212, 1988.2), (214, 2007.6), (216, 2032.2), (218, 2055.6), (220, 2074.2), (222, 2091.6), (224, 2112.4), (226, 2131.8), (228, 2147.8), (230, 2172), (232, 2198.6), (234, 2213.6), (236, 2232.6), (238, 2249.4), (240, 2276.6), (242, 2292.8), (244, 2309.4), (246, 2327), (248, 2349.2), (250, 2365.8), (252, 2388.8), (254, 2405.2), (256, 2425.6), (258, 2440), (260, 2462.8), (262, 2476.6)],
    "T0": [(30, 2474.6), (32, 2464.8), (34, 2446.6), (36, 2426.8), (38, 2397), (40, 2374.2), (42, 2359.6), (44, 2333.6), (46, 2315), (48, 2297.2), (50, 2274.6), (52, 2253.6), (54, 2234), (56, 2210.4), (58, 2184.2), (60, 2161.6), (62, 2141.6), (64, 2120.4), (66, 2100.8), (68, 2071.6), (70, 2055.2), (72, 2036.8), (74, 2016), (76, 1997), (78, 1967.4), (80, 1950), (82, 1928), (84, 1902.2), (86, 1879.8), (88, 1854), (90, 1828), (92, 1806), (94, 1790.6), (96, 1769.6), (98, 1746.2), (100, 1721.2), (102, 1704.4), (104, 1677.4), (106, 1652.2), (108, 1631.8), (110, 1604.6), (112, 1579.2), (114, 1561.2), (116, 1536.4), (118, 1515.4), (120, 1491.8), (122, 1464), (124, 1440), (126, 1418.8), (128, 1397), (130, 1372.2), (132, 1347.2), (134, 1317.6), (136, 1292.6), (138, 1274.8), (140, 1258.2), (142, 1232.4), (144, 1206.4), (146, 1180), (148, 1156.6), (150, 1128.2), (152, 1112.2), (154, 1091.6), (156, 1066.2), (158, 1044.2), (160, 1023), (162, 1006.8), (164, 986.8), (166, 962.8), (168, 946), (170, 925.4), (172, 902.8), (174, 882.6), (176, 864), (178, 836.8), (180, 819.6), (182, 800.8), (184, 775.8), (186, 757.4), (188, 734.4), (190, 712.4), (192, 693.8), (194, 672), (196, 648.6), (198, 626), (200, 610.2), (202, 590.2), (204, 572.2), (206, 551), (208, 530.8), (210, 507.8)],
}

class RobotLegPart:
    def __init__(self, boardChannel, timingSamples, degreesOffset) -> None:
        self.samples = timingSamples

        self.boardChannel = boardChannel

        self.smallestAngle = self.samples[0][0]
        self.largestAngle = self.samples[-1][0]

        self.sampleStep = 2

        self.degreesOffset = degreesOffset

    def setAngle(self, degrees=None, radians=None, us=None):
        if radians is not None:
            degrees = round(math.degrees(radians))
        if degrees is not None:
            degrees += self.degreesOffset
            us = self.msFromDegrees(degrees)

        print("SET {}: {}deg - {}us".format(self.boardChannel, degrees, us))

    def msFromDegrees(self, degrees):
        lowIdx = round((degrees - self.smallestAngle) / self.sampleStep)

        if lowIdx < 0 or lowIdx > len(self.samples) - 2:
            raise Exception("Angle out of bounds")
        
        lowDegree, lowMs = self.samples[lowIdx]
        highDegree, highMs = self.samples[lowIdx + 1]

        fade = (degrees - lowDegree) / (highDegree - lowDegree) # Percent our degree is between the 2 surrounding samples 
        return round((highMs - lowMs) * fade + lowMs)


class RobotLeg:
    def __init__(self, coxa: RobotLegPart, femur: RobotLegPart, tibia: RobotLegPart) -> None:
        self.coxa = coxa
        self.femur = femur
        self.tibia = tibia
        self.allParts = [coxa, femur, tibia]

    def legToPosition(self, x, y, z):
        (coxaRads, femurRads, tibiaRads) = legIkFromCoords(x, y, z)
        self.coxa.setAngle(radians=coxaRads)
        self.femur.setAngle(radians=femurRads)
        self.tibia.setAngle(radians=tibiaRads)
    

# From the top perspective of the leg sitting straight outwards from the body 
# x = distance from coxa servo of tibia end (in/out)
# y: distance left/right of tibia end from coxa servo at neutral position
# z: distance up/down of tibia end from coxa servo
def legIkFromCoords(x, y, z):
    # Ripped from: https://oscarliang.com/inverse-kinematics-and-trigonometry-basics/
    gamma = math.atan(x / y)

    L1 = math.sqrt(x**2 + y**2)
    L = math.sqrt(z ** 2 + (L1 - coxaLength) ** 2)
    a1 = math.acos((z) / L)
    a2 = math.acos(round((tibiaLength ** 2 - femurLength ** 2 - L ** 2) / (-2 * femurLength * L), 4))
    alpha = a1 + a2
    beta = math.acos(round((L ** 2 - tibiaLength ** 2 - femurLength ** 2) / (-2 * tibiaLength * femurLength), 4))

    return (gamma, alpha, beta)



def createLeg(channelGroupNum, legIdx):
    return RobotLeg(
        RobotLegPart((channelGroupNum * 3) + 0, calibrationTimings['C' + str(legIdx)], 180),
        RobotLegPart((channelGroupNum * 3) + 1, calibrationTimings['F' + str(legIdx)], 0),
        RobotLegPart((channelGroupNum * 3) + 2, calibrationTimings['T' + str(legIdx)], 0),
    )

leg = createLeg(0, 0)

import time

y = 100
z = 60
x = -70
incr = 2
leg.legToPosition(x,y,z)
# while True:
#     x += incr
#     if x < -70 or x > 70:
#         incr *= -1
#     leg.legToPosition(x,y,z)

#     time.sleep(0.5)