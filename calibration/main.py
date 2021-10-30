from aruco_process import ArucoProcessor
from typing import Optional
import cv2
from serial.serialutil import SerialException
import serial
import queue
import threading

from joint import COXA, FEMUR, TIBIA
from servo_workflow import JointCalibrationWorkflow


# taken from https://stackoverflow.com/a/54755738
class BufferlessVideoCapture:

  def __init__(self, name):
    self.cap = cv2.VideoCapture(name)
    self.q = queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except queue.Empty:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()


def resizeImage(inputImage, scaleRatio):
    width = int(inputImage.shape[1] * scaleRatio)
    height = int(inputImage.shape[0] * scaleRatio)
    dim = (width, height)
    return cv2.resize(inputImage, dim, interpolation = cv2.INTER_AREA)

def tryStartCalibration(joint, port="COM3", baudrate=9600):
    try:
        hexapodSerial = serial.Serial(port=port, baudrate=baudrate)
        return JointCalibrationWorkflow(joint, hexapodSerial)
    except SerialException as e:
        print("Failed to start calibration workflow, got SerialException: " + str(e))
        return None


if __name__ == "__main__":
    processor = ArucoProcessor()

    isVideo = True
    if isVideo:
        # streamUrl = "http://192.168.0.33/cgi-bin/hi3510/snap.cgi?&-getstream&-chn=1"
        # frameScaleRatio = 1

        streamUrl = "http://192.168.0.107:8080/video"
        frameScaleRatio = 0.6

        stream = BufferlessVideoCapture(streamUrl)
        # stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # if not stream.isOpened():
        #     print ("Cannot open stream")
        #     sys.exit()

        calibrationCross = False
        runningCalibration: Optional[JointCalibrationWorkflow] = None

        while True:
            frame = stream.read()
            # ret, frame = stream.read()
            # if not ret:
            #     print ("Can't receive frame")
            #     break
            
            if frameScaleRatio != 1:
                frame = resizeImage(frame, frameScaleRatio)
            
            processor.processAruco(frame, runningCalibration, calibrationCross)
            
            if runningCalibration is not None:
                runningCalibration.tick()

            cmd = cv2.waitKey(1)
            if cmd == ord('q'):
                break
            elif cmd == ord('c'):
                calibrationCross = not calibrationCross
            elif cmd == ord('i'):
                runningCalibration = tryStartCalibration(FEMUR)
            elif cmd == ord('o'):
                runningCalibration = tryStartCalibration(TIBIA)
            elif cmd == ord('p'):
                runningCalibration = tryStartCalibration(COXA)
            elif cmd == ord('m'):
                if runningCalibration is not None:
                    runningCalibration.start()
            elif cmd == ord('n'):
                if runningCalibration is not None:
                    runningCalibration.printMapping()

        # stream.release()
        cv2.destroyAllWindows()

    else:
        imgpath = "C:\\Users\\chris\\Downloads\\PXL_20210913_092137294.jpg"
        originalImage = cv2.imread(imgpath)

        resizedImage = resizeImage(originalImage, 0.4)
        processor.processAruco(resizedImage, None, True)

        cv2.waitKey(0)
        cv2.destroyAllWindows()
