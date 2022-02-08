import rtde_control
import rtde_receive
import time
import numpy as np
import threading


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

        self.pathThread = threading.Thread(target=self.__executePath)

        self.receiveActive = True
        self.receiveThread = threading.Thread(target=self.__receiveRTDEData)
        self.receiveThread.start()

    def __del__(self):
        self.rtde_c.stopScript()
        self.receiveActive = False
        self.isActive = False
        self.pathThread.join()
        self.receiveThread.join()

    def __receiveRTDEData(self):

        timeToWait = 1.0/self.freq
        while self.receiveActive:
            start = time.time()
            self.pose = self.rtde_r.getActualTCPPose()
            self.actualTcpPose = self.rtde_r.getActualTCPPose()
            self.targetTcpPose = self.rtde_r.getTargetTCPPose()
            self.actualTcpSpeed = self.rtde_r.getActualTCPSpeed()
            self.actualTcpForce = self.rtde_r.getActualTCPPose()
            self.actualQ = self.rtde_r.getActualTCPPose()
            self.actualQd = np.zeros(6)
            self.targetQ = np.zeros(6)
           # self.rtde_r.get
            end = time.time()
            duration = end - start
            if duration < timeToWait:
                time.sleep(timeToWait - duration)
            if self.receiveCallback is not None:
                self.receiveCallback(self.pose)

    def setReceiveCallback(self, callback):
        self.receiveCallback = callback

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
        if not self.pathThread.is_alive():
            self.pathThread = threading.Thread(target=self.__executePath)
            self.pathThread.start()
        else:
            print("Already running!")

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

    def stopPath(self):
        self.isActive = False
        self.rtde_c.servoStop()
        if self.pathThread.is_alive():
            self.pathThread.join()

    def goToPathStart(self):
        if len(self.pathQList) >= 1:
            self.rtde_c.moveJ(self.pathQList[0])
        else:
            print("No path was set!")


if __name__ == "__main__":
    pose1 = np.array([-0.37595091, -0.43348931, 0.29035881, -0.72104235, -2.99956821, -0.01164331])
    pose2 = np.array([0.06584268, -0.57000452, 0.29046544, 0.54908986, -3.03728964, 0.02453679])

    newPath = [pose1, pose2, pose1, pose2]

    rc = RTDE_ur5_communication("192.168.1.159", 50)
    rc.setCurrentPath(newPath)

    rc.startExecution()
    print("aa")
    time.sleep(4)
    rc.stopPath()
    rc.startExecution()
