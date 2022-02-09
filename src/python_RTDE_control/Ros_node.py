import rospy
from python_RTDE_control.RTDE_ur5_communication import RTDE_ur5_communication
from ur5_RTDE_controller.msg import RTDEInfo, RTDETransformedInfo
from ur5_RTDE_controller.srv import StartPath, StopPath, SetNewPathFile, SetTransformPose



class Ros_node():
    def __init__(self, RTDECommunication):
        rospy.init_node('RTDEComm', anonymous=True)

        self.RTDECommunication = RTDECommunication

        self.pub = rospy.Publisher('UR_RTDE_Info', RTDEInfo, queue_size=10)
        self.RTDECommunication.setReceiveCallback(self.receiveCallback)

        self.pubT = rospy.Publisher('UR_RTDE_Transformed_Info', RTDETransformedInfo, queue_size=10)
        self.RTDECommunication.setReceiveTransformedCallback(self.receiveTransformedCallback)

        rospy.Service('StartPath', StartPath, self.handleServiceStartPath)
        rospy.Service('StopPath', StopPath, self.handleServiceStopPath)
        rospy.Service('SetNewPathFile', SetNewPathFile, self.handleServiceSetNewPathFile)
        rospy.Service('SetTransformPose', SetTransformPose, self.handleServiceSetTransformPose)

    def receiveCallback(self, actualTcpPose, targetTcpPose, actualTcpSpeed, actualTcpForce, actualQ, actualQd, targetQ):
        newMsg = RTDEInfo()
        newMsg.actualTcpPose = actualTcpPose
        newMsg.targetTcpPose = targetTcpPose
        newMsg.actualTcpSpeed = actualTcpSpeed
        newMsg.actualTcpForce = actualTcpForce
        newMsg.actualQ = actualQ
        newMsg.actualQd = actualQd
        newMsg.targetQ = targetQ
        self.pub.publish(newMsg)

    def receiveTransformedCallback(self, actualTcpPose):
        newMsg = RTDETransformedInfo()
        newMsg.actualTcpPose = actualTcpPose
        self.pubT.publish(newMsg)

    def handleServiceStartPath(self, req):
        return self.RTDECommunication.startExecution()

    def handleServiceStopPath(self, req):
        return self.RTDECommunication.stopPath()

    def handleServiceSetNewPathFile(self, req):
        return self.RTDECommunication.setCurrentPathFromCSVFile(req.csvPath)

    def handleServiceSetTransformPose(self, req):
        return self.RTDECommunication.setReceiveTransformedFromCSVFile(req.csvPath)


if __name__ == '__main__':
    try:
        rc = RTDE_ur5_communication("192.168.1.159", 50)
        Ros_node(rc)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass