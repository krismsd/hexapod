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


class Leg:
    def __init__(self, legIdx, initialPosition):
        self.legIdx = legIdx

        if not self.updatePosition(initialPosition):
            raise Exception("Unable to initialise leg to position")
        
        self.commitPosition()

    def updatePosition(self, pos):
        success = sendCommand("legik {} {} {} {} 0".format(self.legIdx, *pos))
        if success:
            self.nextPosition = pos
            return True
        else:
            return False

    def commitPosition(self):
        self.position = self.nextPosition
        self.bodyPosition = legToBodySpace(self.legIdx, self.nextPosition)

    def getBodySpaceLiftedHeight(self, groundZ):
        # Get height from ground (in body space) assuming groundZ represents the bodies distance from ground
        return groundZ - self.bodyPosition[2]


numLegs = 6
degreesBetweenLegs = int(360 / numLegs)


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

    LEG_GROUPS = {
        1: [[0], [1], [2], [3], [4], [5]],
        2: [[0, 3], [1, 4], [2, 5]],
        3: [[0, 2, 4], [1, 3, 5]],
    }

    def __init__(self, legs, stablePos, maxLegsToLift):
        self.legs = legs
        self.stablePos = stablePos
        self.maxLegsToLift = maxLegsToLift # Max amount of legs to lift at any one time

        self.strideLength = 20 # Limit of leg reach in X-Y plane
        self.strideHeight = 40 # Amount leg will lift during lifting leg movement

        self.legMoveResolution = 3 # How much we can move leg end effectors per tick for xy-combined or z axis

        self.liftingLegs = []


    def tick(self, walkAngle):
        if walkAngle is not None:
            walkAngleRads = radians(walkAngle)
            cosWalkAngle = cos(walkAngleRads)
            sinWalkAngle = sin(walkAngleRads)

            # TODO: if we're homing then consider ground limit always reached?
            #  and move lifted legs towards stable coords
            #  once lowered if any ground legs not at stable, lift then and rerun

            # This is the offset from our stable coordinates in body space we wish to move
            # the current/next set of air legs to - we'll select legs that are furthest from this 
            # offset to lift next
            airTargetPositionFromStable = (
                round(self.strideLength * cosWalkAngle, 2), # x
                round(self.strideLength * sinWalkAngle, 2), # y
            )

            groundZ = self.findGroundZ()

            self.cleanupGroundedLiftingLegs(groundZ)

            if len(self.liftingLegs) == 0:
                self.findNewLiftingLegs(airTargetPositionFromStable)

            # Calculate our desired body coord offset move amount
            groundLegBodyMoveDelta = (
                -round(self.legMoveResolution * cosWalkAngle, 2), # x
                -round(self.legMoveResolution * sinWalkAngle, 2), # y
                0, # z
            )
            (potentialLegBodySpaceMoves, 
             groundBoundaryBroken, 
             liftedBoundaryBroken) = self.calcLegPotentialMovesAndBoundaries(groundLegBodyMoveDelta, airTargetPositionFromStable)

            finalLegPositions = self.calcFinalLegPositions(potentialLegBodySpaceMoves, groundBoundaryBroken, liftedBoundaryBroken, groundZ)

            self.updateLegs(finalLegPositions)


    def findGroundZ(self):
        groundZ = None
        for legIdx in range(numLegs):
            leg = self.legs[legIdx]
            isLiftingLeg = legIdx in self.liftingLegs
            legBodyPos = leg.bodyPosition

            if not isLiftingLeg:
                if groundZ is None:
                    groundZ = legBodyPos[2]
                    
                if groundZ != legBodyPos[2]:
                    print("WARNING: detected different Z values for ground legs ({} and {}".format(legBodyPos[2], groundZ))
                
                # Leg with highest Z is touching the ground cleanly
                groundZ = max(groundZ, legBodyPos[2])

        return groundZ


    def cleanupGroundedLiftingLegs(self, groundZ):
        # self.liftingLegs = [idx for idx in self.liftingLegs if self.legs[idx].bodyPosition[2] < groundZ]

        cleanedLegs = []
        for legIdx in self.liftingLegs:
            leg = self.legs[legIdx]
            legBodyPos = leg.bodyPosition

            if legBodyPos[2] < groundZ:
                cleanedLegs.append(legIdx)
        
        self.liftingLegs = cleanedLegs


    def findNewLiftingLegs(self, bodyTargetPositionFromStable):
        # Select a new set of legs to be lifted based on which leg is furthest away from the target positon
        # Taget position can be imaged as x,y on circle with full stride distance and current walk angle
        # i.e. if we're continueing to walk forward then the legs at the back should be selected to bring forward

        # Find the leg furthest from the target position
        furthestLegIdx = None
        furthestLegDistance = None
        for legIdx in range(numLegs):
            leg = self.legs[legIdx]
            legBodyPos = leg.bodyPosition
            stableBodyPos = legToBodySpace(legIdx, self.stablePos)

            stableOffsetFromTarget = (
                legBodyPos[0] - stableBodyPos[0] - bodyTargetPositionFromStable[0],
                legBodyPos[1] - stableBodyPos[1] - bodyTargetPositionFromStable[1],
            )

            distanceFromTarget = coordDistance(stableOffsetFromTarget[0], stableOffsetFromTarget[1])
            if furthestLegDistance is None or distanceFromTarget > furthestLegDistance:
                furthestLegIdx = legIdx
                furthestLegDistance = distanceFromTarget

        # Decide which legs we want to lift based on current `maxLegsToLift`
        self.liftingLegs = next(group for group in self.LEG_GROUPS[self.maxLegsToLift] if furthestLegIdx in group)


    def calcLegPotentialMovesAndBoundaries(self, groundLegBodyMoveDelta, airTargetPositionFromStable):
        # Calculate potential leg target xy and boundary break conditions 
        liftedBrokenBoundaries = []
        groundBrokenBoundaries = []
        potentialLegBodySpaceMoves = []

        for legIdx in range(numLegs):
            leg = self.legs[legIdx]
            isLiftingLeg = legIdx in self.liftingLegs
            legBodyPos = leg.bodyPosition
            stableBodyPos = legToBodySpace(legIdx, self.stablePos)
            
            # The ground legs should move in reverse to push the body forward
            # while the lifted legs should move reach forward
            # offsetDeltaMulti = 1 if isLiftingLeg else -1

            potentialMove = None
            if isLiftingLeg:
                # If air leg we should move towards the target position
                # Taking the distance to the stable position, then take the negative to get the delta
                #  that will take us to the target 
                deltaXFromTarget = -(legBodyPos[0] - stableBodyPos[0] - airTargetPositionFromStable[0])
                deltaYFromTarget = -(legBodyPos[1] - stableBodyPos[1] - airTargetPositionFromStable[1])
                distanceFromTarget = coordDistance(deltaXFromTarget, deltaYFromTarget)

                # The leg positions are only int's so if we're close enough consider the target reached
                # otherwise we'll keep jumping over the target
                distanceFromTarget = round(distanceFromTarget)

                if distanceFromTarget == 0:
                    potentialMove = (0, 0, 0)
                else:
                    # Ensure we move at most legMoveResolution, or just cover the delta if we're close enough
                    scaleForMove = min(self.legMoveResolution / distanceFromTarget, 1)
                    potentialMove = (
                        scaleForMove * deltaXFromTarget,
                        scaleForMove * deltaYFromTarget,
                        0 # Lifted leg z is handled afterwards
                    )

            else:
                # If ground leg then we should move away from the moveAngle
                potentialMove = groundLegBodyMoveDelta

            potentialLegBodySpaceMoves.append(potentialMove)

            # See if potential move goes outside of strideLength and report boundary breaks
            newDistanceFromStable = coordDistance(
                legBodyPos[0] - stableBodyPos[0] + potentialMove[0],
                legBodyPos[1] - stableBodyPos[1] + potentialMove[1]
            )

            # Leg has broken boundary if leg will be over-extended and the leg is extending further
            # i.e. If leg is moving towards a more stable position then dont consider the boundary broken
            boundaryBroken = False
            if newDistanceFromStable + 1 >= self.strideLength:
                currentDistanceFromStable = coordDistance(
                    legBodyPos[0] - stableBodyPos[0],
                    legBodyPos[1] - stableBodyPos[1]
                )

                boundaryBroken = newDistanceFromStable >= currentDistanceFromStable

            if isLiftingLeg:
                liftedBrokenBoundaries.append(boundaryBroken)
                # liftedBoundaryBroken = liftedBoundaryBroken and boundaryBroken
            else:
                groundBrokenBoundaries.append(boundaryBroken)
                # groundBoundaryBroken = groundBoundaryBroken or boundaryBroken

        # Ground boundary is broken if any ground leg can't move
        groundBoundaryBroken = any(groundBrokenBoundaries)

        # Lifting boundary is broken if all lifted legs can't move
        liftedBoundaryBroken = all(liftedBrokenBoundaries)

        return (
            potentialLegBodySpaceMoves,
            groundBoundaryBroken,
            liftedBoundaryBroken,
        )


    def calcFinalLegPositions(self, potentialLegXYBodySpaceMoves, groundBoundaryBroken, liftedBoundaryBroken, groundZ):
        
        # Detect if any lifted legs are in the process of dropping/raising
        # Assumed to be if they're not on the ground or not at stride height
        isLiftedMidTransition = False
        for legIdx in self.liftingLegs:
            leg = self.legs[legIdx]
            liftedHeight = leg.getBodySpaceLiftedHeight(groundZ)
            if liftedHeight < self.strideHeight and liftedHeight >= 0:
                isLiftedMidTransition = True
                break

        finalLegPositions = []
        for legIdx in range(numLegs):
            leg = self.legs[legIdx]
            isLiftingLeg = legIdx in self.liftingLegs
            potentialBodySpaceDelta = potentialLegXYBodySpaceMoves[legIdx]

            bodyDelta = None
            if isLiftedMidTransition or (groundBoundaryBroken and liftedBoundaryBroken):
                # Lifted legs are already dropping/raising or we've exceeded all bounds
                deltaZ = 0 # Note - Do nothing to ground legs while dropping/raising
                if isLiftingLeg:
                    liftedHeight = leg.getBodySpaceLiftedHeight(groundZ)

                    if liftedBoundaryBroken:
                        # Lower leg - inrease Z value
                        deltaZ = min(self.legMoveResolution, liftedHeight)  
                    else:
                        # Raise leg - decrease Z value
                        deltaZ = -min(self.legMoveResolution, self.strideHeight - liftedHeight)
                
                bodyDelta = (0, 0, deltaZ)

            else:
                # Otherwise apply potential moves depending on boundary conditions
                if (isLiftingLeg and not liftedBoundaryBroken) or (not isLiftingLeg and not groundBoundaryBroken):
                    bodyDelta = potentialBodySpaceDelta
                else:
                    bodyDelta = (0, 0, 0)

            legDelta = bodyToLegSpace(legIdx, bodyDelta)
            finalLegPositions.append((
                round(leg.position[0] + legDelta[0]),
                round(leg.position[1] + legDelta[1]),
                round(leg.position[2] + legDelta[2]),
            ))

        return finalLegPositions


    def updateLegs(self, finalLegPositions):
        canMove = True

        for legIdx in range(numLegs):
            leg = self.legs[legIdx]
            updateSuccess = leg.updatePosition(finalLegPositions[legIdx])

            canMove = canMove and updateSuccess

        # If all move commands succeeded then commit the commits to hexapod and local leg data
        if canMove:
            sendCommand("commit -1")

            # Commit our local position as well - not needed when we move the controller onto the hexapod itself
            for legIdx in range(numLegs):
                leg = self.legs[legIdx]
                leg.commitPosition()



import time
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

##########

initialStablePos = (0, 120, 100)

# Initialise legs
# Do it slowly just in case
print("Initialising legs")
legs = []
for i in range(6):
    leg = Leg(i, initialStablePos)
    leg.commitPosition()
    legs.append(leg)

    time.sleep(0.5)


walker = WalkController(legs, initialStablePos, 3)
print("Controller initialised - ready for command")

walkerWalkAngle = None
walkerWalkSpeed = None

def commandCallback(command):
    global walkerWalkAngle
    print("Got command: " + command)

    if command.isdigit():
        angle = int(command)
        if angle >= 0 and angle <= 360:
            print("Got valid angle")
            walkerWalkAngle = angle
        else:
            print("Invalid angle")
    elif command == "z":
        print("Got clear angle command")
        walkerWalkAngle = None
    elif command == "q":
        print("Got quit command")
        return False
    else:
        print("Invalid command")

    return True


#start the Keyboard thread
kthread = KeyboardThread(commandCallback)








from aiohttp import web, WSMsgType
from pathlib import Path

async def createServer():
    app = web.Application()
    routes = web.RouteTableDef()

    @routes.get('/')
    async def getIndex(request):
        return web.FileResponse(Path(__file__).parent.joinpath("index.html"))

    @routes.get('/ws')
    async def websocket(request):
        global walkerWalkAngle, walkerWalkSpeed

        ws = web.WebSocketResponse()
        await ws.prepare(request)

        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                if msg.data == 'close':
                    await ws.close()
                elif msg.data[:2] == "a:":
                    parts = msg.data[2:].split(",")
                    walkerWalkAngle = int(parts[0])
                    walkerWalkSpeed = float(parts[1])
                    await ws.send_str('ok')
                elif msg.data[:2] == "s:":
                    walkerWalkAngle = None
                    walkerWalkSpeed = None
                else:
                    await ws.send_str('unknown command')
            elif msg.type == WSMsgType.ERROR:
                print('ws connection closed with exception %s' % ws.exception())

        return ws

    app.add_routes(routes)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, 'localhost', 5000)
    await site.start()


# web.run_app(app, host='0.0.0.0', port=5000)





# from flask import Flask, request, jsonify, send_file

# app = Flask(__name__)

# @app.route('/', methods=['GET'])
# def index():
#     return send_file('index.html')

# @app.route('/walk', methods=['POST'])
# def setWalk():
#     data = request.get_json()
#     if data is None:
#         raise Exception("No json provided")

#     global walkerWalkAngle, walkerWalkSpeed
#     walkerWalkAngle = data['angle']
#     walkerWalkSpeed = data['speed']
#     return jsonify({"result":"ok"})


# serverThread = threading.Thread(target=lambda: app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)).start()



# import asyncio
# import websockets
# import json

# async def wsHandler(websocket, path):
#     async for message in websocket:
#         data = json.loads(message)
#         walkerWalkAngle = data['angle']
#         walkerWalkSpeed = data['speed']        


# async def wsMain():
#     async with websockets.serve(wsHandler, "localhost", 8765):
#         await asyncio.Future()  # run forever

# asyncio.run


import asyncio

async def createWalkerLoop():
    while True:
        walker.tick(walkerWalkAngle)
        await asyncio.sleep(0)


async def main():
    await asyncio.gather(
        createServer(),
        createWalkerLoop()
    )


# while kthread.is_alive():
#     # print("angle: {}, speed: {}".format(walkerWalkAngle, walkerWalkSpeed))
#     walker.tick(walkerWalkAngle)

asyncio.run(main())


kthread.join()
