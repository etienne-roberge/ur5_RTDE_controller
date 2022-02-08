import rospy
from python_RTDE_control.RTDE_ur5_communication import RTDE_ur5_communication
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ur5_RTDE_controller.msg import RTDEInfo
from ur5_RTDE_controller.srv import StartPath, StopPath, SetNewPathFile



class Ros_node():
    def __init__(self, RTDECommunication):
        rospy.init_node('RTDEComm', anonymous=True)

        self.RTDECommunication = RTDECommunication
        self.pub = rospy.Publisher('UR_RTDE_Info', RTDEInfo, queue_size=10)
        self.rate = rospy.Rate(10)  # 10hz
        self.RTDECommunication.setReceiveCallback(self.receiveCallback)

        rospy.Service('StartPath', StartPath, self.handleServiceStartPath)
        rospy.Service('StopPath', StopPath, self.handleServiceStopPath)
        rospy.Service('SetNewPathFile', SetNewPathFile, self.handleServiceSetNewPathFile)

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

    def handleServiceStartPath(self, req):
        return self.RTDECommunication.startExecution()

    def handleServiceStopPath(self, req):
        return self.RTDECommunication.stopPath()

    def handleServiceSetNewPathFile(self, req):
        return self.RTDECommunication.setCurrentPathFromCSVFile(req.csvPath)


if __name__ == '__main__':
    try:
        #talker()
        rc = RTDE_ur5_communication("192.168.1.159", 50)
        Ros_node(rc)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass