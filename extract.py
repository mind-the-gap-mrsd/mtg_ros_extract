import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from scipy.spatial.transform import Rotation as R
import os

os.system("rm bot.kitti vicon_gt.kitti")

def callPoseMsg(msg):
    px, py, pz = msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z
    qx, qy, qz, qw = msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w
    print("here")
    print(px,py,pz)
    r = R.from_quat([qx, qy, qz, qw]).as_matrix()
    T = np.vstack((np.hstack((r, np.array([px, py, 0]).reshape((3,1)))))).reshape((1,12))
    np.savetxt(vicon_gt, T)

def callOdomMsg(msg):
    px, py, pz = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
    qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
    r = R.from_quat([qx, qy, qz, qw]).as_matrix()
    T = np.vstack((np.hstack((r, np.array([px, py, 0]).reshape((3,1)))))).reshape((1,12))
    np.savetxt(bot_odom, T)

def callPose(odomMsg, poseMsg):
    callPoseMsg(poseMsg)
    callOdomMsg(odomMsg)

def callPoseTemp(poseMsg):
    callPoseMsg(poseMsg)
    # callOdomMsg(odomMsg)

def loc_extract():
    botSub = Subscriber("mtg_agent_bringup_node/agent_0/odom_data_quat", Odometry)
    viconSub = Subscriber("vicon/k117/k117", TransformStamped)
    ats = ApproximateTimeSynchronizer([botSub, viconSub], queue_size=1, slop=0.2)
    # ats = ApproximateTimeSynchronizer([viconSub], queue_size=1, slop=0.2)
    ats.registerCallback(callPoseTemp)
    # ats.registerCallback(callPoseTemp)


if __name__ == '__main__':
    rospy.init_node("main", anonymous=True)

    vicon_gt = open("vicon_gt.kitti", "ab")
    bot_odom = open("bot.kitti", "ab")
    
    loc_extract()
    
    rospy.spin()