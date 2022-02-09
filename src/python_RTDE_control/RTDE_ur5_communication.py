import rtde_control
import rtde_receive
import time
import numpy as np
import threading
import csv
import os.path
import signal

from scipy.spatial.transform import Rotation as R


class RTDE_ur5_communication:
    def __init__(self, ip, freq):
        self.rtde_c = rtde_control.RTDEControlInterface(ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(ip)

        # Parameters
        self.velocity = 0.5
        self.acceleration = 0.5
        self.dt = 1.0 / 500  # 2ms
        self.lookahead_time = 0.1
        self.gain = 300

        self.freq = freq

        self.currentPath = None
        self.pathQList = None

        self.isActive = False

        self.receiveCallback = None
        self.actualTcpPose = np.zeros(6)
        self.targetTcpPose = np.zeros(6)
        self.actualTcpSpeed = np.zeros(6)
        self.actualTcpForce = np.zeros(6)
        self.actualQ = np.zeros(6)
        self.actualQd = np.zeros(6)
        self.targetQ = np.zeros(6)

        self.receiveTransformedCallback = None
        self.transformPose = np.zeros(6)

        self.pathThread = threading.Thread(target=self.__executePath)

        self.receiveActive = True
        self.receiveThread = threading.Thread(target=self.__receiveRTDEData)
        self.receiveThread.start()

        signal.signal(signal.SIGINT, self.__SIGINT_HANDLER)

    def __del__(self):
        self.rtde_c.stopScript()
        self.receiveActive = False
        self.isActive = False
        self.pathThread.join()
        self.receiveThread.join()

    def __SIGINT_HANDLER(self, signum, frame):
        self.receiveActive = False
        self.isActive = False
        if self.pathThread.is_alive():
            self.pathThread.join()
        if self.receiveThread.is_alive():
            self.receiveThread.join()

    def __receiveRTDEData(self):
        timeToWait = 1.0 / self.freq
        while self.receiveActive:
            start = time.time()
            self.actualTcpPose = self.rtde_r.getActualTCPPose()
            self.targetTcpPose = self.rtde_r.getTargetTCPPose()
            self.actualTcpSpeed = self.rtde_r.getActualTCPSpeed()
            self.actualTcpForce = self.rtde_r.getActualTCPForce()
            self.actualQ = self.rtde_r.getActualQ()
            self.actualQd = self.rtde_r.getActualQd()
            self.targetQ = self.rtde_r.getTargetQ()
            end = time.time()
            duration = end - start
            if duration < timeToWait:
                time.sleep(timeToWait - duration)
            if self.receiveCallback is not None:
                self.receiveCallback(self.actualTcpPose,
                                     self.targetTcpPose,
                                     self.actualTcpSpeed,
                                     self.actualTcpForce,
                                     self.actualQ,
                                     self.actualQd,
                                     self.targetQ)

            if self.receiveTransformedCallback is not None:
                self.receiveTransformedCallback(transformPose(self.transformPose, self.actualTcpPose))

    def setReceiveCallback(self, callback):
        self.receiveCallback = callback

    def setReceiveTransformedCallback(self, callback):
        self.receiveTransformedCallback = callback

    def setReceiveTransformedFromCSVFile(self, filename):
        if not os.path.isfile(filename):
            print("File not found!")
            return False

        newPose = []

        try:
            with open(filename, newline='') as csvfile:
                csvReader = csv.reader(csvfile, delimiter=',', quotechar='|')
                for row in csvReader:
                    if len(row) == 6:
                        newPose = np.array([float(row[0]), float(row[1]), float(row[2]),
                                            float(row[3]), float(row[4]), float(row[5])])
        except IOError:
            print("Can't open csv file!")
            return False
        except ValueError:
            print("Error reading values in csv file!")
            return False

        if len(newPose) == 0:
            print("No pose in file")
            return False

        print(newPose)
        self.transformPose = newPose

        return True

    def setCurrentPathFromCSVFile(self, filename):
        if not os.path.isfile(filename):
            print("File not found!")
            return False

        newPath = []

        try:
            with open(filename, newline='') as csvfile:
                csvReader = csv.reader(csvfile, delimiter=',', quotechar='|')
                for row in csvReader:
                    if len(row) == 6:
                        newPath.append(np.array([float(row[0]), float(row[1]), float(row[2]),
                                                 float(row[3]), float(row[4]), float(row[5])]))
        except IOError:
            print("Can't open csv file!")
            return False
        except ValueError:
            print("Error reading values in csv file!")
            return False

        if len(newPath) == 0:
            print("No poses in path")
            return False

        print(newPath)
        self.setCurrentPath(newPath)

        return True

    def setCurrentPath(self, path):
        # check if path is a list
        if isinstance(path, list):
            self.currentPath = path
            self.pathQList = []
            # for each pose in path, get the inverse kin
            for pose in path:
                self.pathQList.append(np.array(self.rtde_c.getInverseKinematics(pose)))
        else:
            print("path needs to be a list of pose")

    def startExecution(self):
        if self.currentPath is None:
            print("No path to execute!")
            return False

        if not self.pathThread.is_alive():
            self.pathThread = threading.Thread(target=self.__executePath)
            self.pathThread.start()
        else:
            print("Already running!")
            return False

        return True

    def __executePath(self):
        self.isActive = True
        self.goToPathStart()
        # to do --- generate the speed
        n = 1000
        previousQ = self.pathQList[0]
        for q in self.pathQList:

            if not self.isActive:
                return

            if not (previousQ == q).all():
                qDiff = q - previousQ

                # Execute 500Hz control loop for 2 seconds, each cycle is 2ms
                for i in range(n):

                    if not self.isActive:
                        return

                    newQ = previousQ + (qDiff / n) * i
                    start = time.time()
                    self.rtde_c.servoJ(newQ, self.velocity, self.acceleration, self.dt, self.lookahead_time, self.gain)
                    end = time.time()
                    duration = end - start
                    if duration < self.dt:
                        time.sleep(self.dt - duration)

            previousQ = q

        self.isActive = False
        self.rtde_c.servoStop()

    def stopPath(self):
        self.isActive = False
        self.rtde_c.servoStop()
        if self.pathThread.is_alive():
            self.pathThread.join()

        return True

    def goToPathStart(self):
        if len(self.pathQList) >= 1:
            self.rtde_c.moveJ(self.pathQList[0])
        else:
            print("No path was set!")


def transformPose(featurePose, poseToTransform):
    f_b = featurePose
    x_b = poseToTransform

    x_f_inBase = f_b - x_b
    translationPart = np.array([x_f_inBase[0], x_f_inBase[1], x_f_inBase[2]])

    f_b_rot = (R.from_rotvec([f_b[3], f_b[4], f_b[5]])).as_matrix()
    f_b_inv = np.linalg.inv(f_b_rot)

    rotPart = (R.from_rotvec([x_b[3], x_b[4], x_b[5]])).as_matrix()

    newTrans = np.matmul(f_b_inv, translationPart)
    newRot = np.matmul(f_b_inv, rotPart)
    newRot = (R.from_matrix(newRot)).as_rotvec()
    newRot = [newRot[1], newRot[2], newRot[0]]

    p = np.array([newTrans[0], newTrans[1], newTrans[2], newRot[0], newRot[1], newRot[2]])

    return p


if __name__ == "__main__":
    # pose1 = np.array([-0.37595091, -0.43348931, 0.29035881, -0.72104235, -2.99956821, -0.01164331])
    # pose2 = np.array([0.06584268, -0.57000452, 0.29046544, 0.54908986, -3.03728964, 0.02453679])
    #
    # newPath = [pose1, pose2, pose1, pose2]
    #
    # rc = RTDE_ur5_communication("192.168.1.159", 50)
    # rc.setCurrentPath(newPath)
    #
    # rc.startExecution()
    # print("aa")
    # time.sleep(4)
    # rc.stopPath()
    # rc.startExecution()

    testTransform = np.array([-0.26483, -0.64482, 0.0663, 1.019, -2.932, -0.031])
    testTransform1 = np.array([-0.26483, -0.64482, 0.0763, 1.019, -2.932, -0.031])
    test = transformPose(testTransform, testTransform1)

    # rc = RTDE_ur5_communication("192.168.1.159", 50)
    # rc.setCurrentPath(testTransform)
    # rc.setCurrentPathFromCSVFile("/home/etienne/Desktop/iros2022_ws/src/ur5_RTDE_controller/test_pathFile.csv")
    # rc.startExecution()

    rtde_c = rtde_control.RTDEControlInterface("192.168.1.159")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.159")
