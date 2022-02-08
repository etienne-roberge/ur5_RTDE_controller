import rospy
from RTDE_ur5_communication import RTDE_ur5_communication
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ur5_RTDE_controller.msg import RTDEInfo



class Ros_node():
    def __init__(self, RTDECommunication):
        rospy.init_node('RTDEComm', anonymous=True)

        self.RTDECommunication = RTDECommunication
        self.pub = rospy.Publisher('UR_RTDE_Info', RTDEInfo, queue_size=10)
        self.rate = rospy.Rate(10)  # 10hz
        self.RTDECommunication.setReceiveCallback(self.receiveCallback)

    def receiveCallback(self, pose):
        newMsg = RTDEInfo()
        newMsg.actualTcpPose = pose
        self.pub.publish(newMsg)



if __name__ == '__main__':
    try:
        #talker()
        rc = RTDE_ur5_communication("192.168.1.159", 50)
        Ros_node(rc)
    except rospy.ROSInterruptException:
        pass