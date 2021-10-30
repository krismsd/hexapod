from math import cos, degrees, radians, sin, sqrt, tan
import serial
import time

hexapod = serial.Serial(port="COM3", baudrate=9600)

def sendCommand(text):
    hexapod.write((text + "\n").encode('ascii'))
    r = hexapod.readline().decode("utf-8").rstrip()
    if r != "ack":
        print("Command failed - input: {}".format(text))
        print("Response: {}".format(r))
        return False
    return True

# class Leg:
#     def __init__(self, legIdx):
#         self.legIdx = legIdx

#     def setPosition(self, pos):
#         self.pos = pos
#         r = sendCommand("legik {} {} {} {} 0".format(self.legIdx, *pos))
#         if r != "ack":
#             print("Unable to move leg to position. leg: {}, x: {}, y: {}, z: {}".format(self.legIdx, *pos))
#             return False

#         return True

#     def getPosition(self):
#         return self.pos



degreesBetweenLegs = 60

sinCache = [sin(radians(i * degreesBetweenLegs)) for i in range(6)]
cosCache = [cos(radians(i * degreesBetweenLegs)) for i in range(6)]


# TODO: legToBody and bodyToLeg are identical save for -/+ sintheta

def legToBodySpace(legIdx, point):
    x, y, z = point

    costheta = cosCache[legIdx]
    sintheta = -sinCache[legIdx]
    return (
        round(x * costheta - y * sintheta, 2),
        round(y * costheta + x * sintheta, 2),
        z # will only change once we consider body roll/pitch
    )

def bodyToLegSpace(legIdx, point):
    x, y, z = point

    costheta = cosCache[legIdx]
    sintheta = sinCache[legIdx]
    return (
        round(x * costheta - y * sintheta, 2),
        round(y * costheta + x * sintheta, 2),
        z # will only change once we consider body roll/pitch
    )

def coordDistance(x, y):
    return sqrt(x ** 2 + y ** 2)



class WalkController:
    LEGSET_MAPPING = {
        # 1: [[0, 1, 2, 3, 4, 5]],
        2: [[0, 2, 4], [1, 3, 5]],
        3: [[0, 3], [1, 4], [2, 5]],
        6: [[0], [1], [2], [3], [4], [5]],
    }

    def __init__(self, numLegsets):
        self.legsets = self.LEGSET_MAPPING[numLegsets]

        self.walkAngle = None # direction of walk in degrees

        self.stableLegXPos = 0 # X pos of leg in stable position
        self.stableLegYPos = 120 # Y pos of leg in stable position
        self.stableLegZPos = 50 # Z pos of leg in stable position

        self.strideLength = 25 # Limit of leg reach in X-Y plane
        self.strideHeight = 40 # Amount leg will lift during lifting leg movement

        self.legMoveResolution = 1 # How much we can move leg end effectors per tick for xy-combined or z axis


        self.legsetOffsets = [(0, 0) for i in range(numLegsets)]

        self.liftingLegset = None
        self.liftedHeight = 0


    def setWalkAngle(self, angle):
        self.walkAngle = angle

    def tick(self):
        # ! Changes to make:
        # remove offsets and base logic on actual leg positions
        # calc heuristics for each leg individually
        # remove legsets - track individual lifting legs
        #  this is to allow lifting legs to change without reconstructing the controller
        #  lifting legs should be decided by findNewLiftingLegs given the current gait option
        #  i.e. this'll mean if we change gait in middle of stride then the current lifted legs will move as previously
        #       but next time we decide lifted legs it'll be based on new gait
        # stableLegNPos's should be changable as well
        #   calcNewState should handle this change
        #   ground legs should always be locked relative to each other
        #     i.e. they should always move equally in body space, or not move if any arent able to move 
        #   air legs should be able to individually hunt their new pos
        #   if air legs are dropping then heuristics should decide their not landing on their stable pos and relift the legs
        #      (if lifting they're already not at their target pos so its irrelevant)
        #   e.g. if stableZPos changes then the ground legs should adapt to this on the fly
        #           (but they shouldnt respond to stableX/YPos changes - they'll be sorted on their next air cycle)
        #   e.g. if stableX/YPos changes then air legs should start hunting this new pos
        # this'll require use of legToBodySpace
        # this'll enable us to add body pitch/roll much more easily later on


        if self.walkAngle is not None:
            # TODO: if we're homing then consider ground limit always reached?
            #  and move lifted legs towards stable coords
            #  once lowered if any ground legs not at stable, lift then and rerun

            if self.liftingLegset is None:
                self.findNewLiftingLegset(self.walkAngle)

            (legsetTargetOffsets, groundBoundaryBroken, liftedBoundaryBroken) = self.calcMoveHeuristics(self.walkAngle)

            self.calcNewState(legsetTargetOffsets, groundBoundaryBroken, liftedBoundaryBroken)

            self.updateLegs()


    def findNewLiftingLegset(self, walkAngle):
        # Select a new legset to be lifted based on which legset is furthest away from the target positon
        # Taget position can be imaged as x,y on circle with full stride distance and current walk angle
        # i.e. if we're continueing to walk forward then the legs at the back should be selected to bring forward
        targetPosition = (
            round(self.strideLength * cos(radians(walkAngle)), 2), # x
            round(self.strideLength * sin(radians(walkAngle)), 2), # y
        )

        furthestLegset = None
        furthestLegsetDistance = None
        for legsetIdx in range(len(self.legsets)):
            legsetOffset = self.legsetOffsets[legsetIdx]

            distanceFromTarget = coordDistance(legsetOffset[0] - targetPosition[0], legsetOffset[1] - targetPosition[1])
            if furthestLegsetDistance is None or distanceFromTarget > furthestLegsetDistance:
                furthestLegset = legsetIdx
                furthestLegsetDistance = distanceFromTarget

            self.liftingLegset = furthestLegset


    def calcMoveHeuristics(self, walkAngle):
        # Calculate our desired body coord offset move amount
        moveDelta = (
            round(self.legMoveResolution * cos(radians(walkAngle)), 2), # x
            round(self.legMoveResolution * sin(radians(walkAngle)), 2), # y
        )

        # Calculate potential legsets target xy and boundary break conditions 
        liftedBoundaryBroken = False
        groundBoundaryBroken = False
        legsetTargetOffsets = []
        for legsetIdx in range(len(self.legsets)):
            isLiftingLegset = self.liftingLegset == legsetIdx
            offsetDeltaMulti = 1 if isLiftingLegset else -1

            legsetOffset = self.legsetOffsets[legsetIdx]

            # Calculate potential next position (used when boundary conditions satisfied)
            targetOffset = (
                legsetOffset[0] + (moveDelta[0] * offsetDeltaMulti),
                legsetOffset[1] + (moveDelta[1] * offsetDeltaMulti),
            )
            legsetTargetOffsets.append(targetOffset)

            currentOffsetDistance = coordDistance(legsetOffset[0], legsetOffset[1])
            newOffsetDistance = coordDistance(targetOffset[0], targetOffset[1])

            # Legset has broken boundary if leg will be over-extended and the legset is extending further
            # i.e. If legset is moving towards a more stable position then dont consider the boundary broken
            boundaryBroken = newOffsetDistance > self.strideLength and newOffsetDistance > currentOffsetDistance

            if isLiftingLegset:
                liftedBoundaryBroken = boundaryBroken
            else:
                groundBoundaryBroken = groundBoundaryBroken or boundaryBroken

        return (legsetTargetOffsets, groundBoundaryBroken, liftedBoundaryBroken)


    def calcNewState(self, legsetTargetOffsets, groundBoundaryBroken, liftedBoundaryBroken):
        # Dropping legs
        if liftedBoundaryBroken and self.liftedHeight > 0:
            self.liftedHeight -= self.legMoveResolution
            if self.liftedHeight <= 0:
                # If the legset is on the ground, clear lifting legset states
                self.liftedHeight = 0
                self.liftingLegset = None

        # Lifting legs
        elif not liftedBoundaryBroken and self.liftedHeight < self.strideHeight:
            self.liftedHeight += self.legMoveResolution

        else:
            for legsetIdx in range(len(self.legsets)):
                if (self.liftingLegset == legsetIdx and not liftedBoundaryBroken) or \
                    (self.liftingLegset != legsetIdx and not groundBoundaryBroken):
                    self.legsetOffsets[legsetIdx] = legsetTargetOffsets[legsetIdx]


    def updateLegs(self):
        canMove = True
        for legsetIdx in range(len(self.legsets)):
            legset = self.legsets[legsetIdx]
            legsetOffsets = self.legsetOffsets[legsetIdx]

            # z axis is down from body whereas height is defined as up from ground
            # so any height offset is applied negative in z axis
            isLifted = legsetIdx == self.liftingLegset
            zOffset = -self.liftedHeight if isLifted else 0

            bodySpaceOffset = (
                legsetOffsets[0],
                legsetOffsets[1],
                zOffset,
            )

            for legIdx in legset:
                # Calculate the offset coordinates in leg space and apply on top of legs stable coords
                legOffset = bodyToLegSpace(legIdx, bodySpaceOffset)

                pos = (
                    round(self.stableLegXPos + legOffset[0]),
                    round(self.stableLegYPos + legOffset[1]),
                    round(self.stableLegZPos + legOffset[2]),
                )

                canMove = canMove and sendCommand("legik {} {} {} {} 0".format(legIdx, *pos))

        if canMove:
            sendCommand("commit -1")



import time
# import sys
# import msvcrt

# num = 0
# done = False
# while not done:
#     print(num)
#     num += 1

#     if msvcrt.kbhit():
#         # print("you pressed",msvcrt.getch(),"so now i will quit")

#         done = True

import threading

class KeyboardThread(threading.Thread):

    def __init__(self, input_cbk, name='keyboard-input-thread'):
        self.input_cbk = input_cbk
        super(KeyboardThread, self).__init__(name=name)
        self.start()

    def run(self):
        while True:
            r = self.input_cbk(input()) #waits to get input + Return
            if not r:
                break

command = ""
def my_callback(inp):
    global command
    command = inp
    return command != "q"

#start the Keyboard thread
kthread = KeyboardThread(my_callback)


walker = WalkController(3)


# Reset legs to optimal pos
for i in range(6):
    sendCommand("legik {} {} {} {} 0".format(i, walker.stableLegXPos, walker.stableLegYPos, walker.stableLegZPos))
    sendCommand("commit -1")

    time.sleep(0.5)



# walker.setWalkAngle(30)

# inputBuffer = ""
while True:
    walker.tick()

    if command != "":
        print("Got command: " + command)

        if command.isdigit():
            angle = int(command)
            if angle >= 0 and angle <= 360:
                print("Got valid angle")
                walker.setWalkAngle(angle)
            else:
                print("Invalid angle")
        elif command == "z":
            print("Got clear angle command")
            walker.setWalkAngle(None)
        elif command == "q":
            print("Got quit command")
            break
        else:
            print("Invalid command")
        command = ""

kthread.join()