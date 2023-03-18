import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import tf

# All
def callback(robot, vicon):
    try:
        # SLAM Toolbox
        (trans, rot) = slamTf.lookupTransform("/map", "/agent_0/base_link", rospy.Time(0))
        print(trans, rot)
        r = R.from_quat(rot).as_matrix()
        T = np.vstack((np.hstack((r, np.array([trans[0], trans[1], 0]).reshape((3,1)))))).reshape((1,12))
        np.savetxt(slam, T, delimiter=' ')

        # VICON
        px, py, pz = vicon.transform.translation.x, vicon.transform.translation.y, vicon.transform.translation.z
        qx, qy, qz, qw = vicon.transform.rotation.x, vicon.transform.rotation.y, vicon.transform.rotation.z, vicon.transform.rotation.w
        print("here")
        print(px,py,pz)
        r = R.from_quat([qx, qy, qz, qw]).as_matrix()
        T = np.vstack((np.hstack((r, np.array([px, py, 0]).reshape((3,1)))))).reshape((1,12))
        np.savetxt(vicon_gt, T, delimiter=' ')        

        # Robot
        px, py, pz = robot.pose.pose.position.x, robot.pose.pose.position.y, robot.pose.pose.position.z
        qx, qy, qz, qw = robot.pose.pose.orientation.x, robot.pose.pose.orientation.y, robot.pose.pose.orientation.z, robot.pose.pose.orientation.w
        r = R.from_quat([qx, qy, qz, qw]).as_matrix()
        T = np.vstack((np.hstack((r, np.array([px, py, 0]).reshape((3,1)))))).reshape((1,12))
        np.savetxt(bot_odom, T, delimiter=' ')
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("skipped?")
        return

# Robot
# def callOdomMsg(msg):
#     px, py, pz = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
#     qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
#     r = R.from_quat([qx, qy, qz, qw]).as_matrix()
#     T = np.vstack((np.hstack((r, np.array([px, py, 0]).reshape((3,1)))))).reshape((1,12))
#     np.savetxt(bot_odom, T, delimiter=' ')

# # def callTf(msg):
# #     try:
# #         (trans, rot) = 

# def callPose(odomMsg, poseMsg):
#     callPoseMsg(poseMsg)
#     callOdomMsg(odomMsg)
#     # callTf(tfs)

# def callPoseTemp(poseMsg):
#     # callPoseMsg(poseMsg)
#     callOdomMsg(odomMsg)

def loc_extract():
    botSub = Subscriber("mtg_agent_bringup_node/agent_0/odom_data_quat", Odometry)
    viconSub = Subscriber("vicon/k114/k114", TransformStamped)
    # slamTf = tf.TransformListener()
    ats = ApproximateTimeSynchronizer([botSub, viconSub], queue_size=1, slop=1)
    # ats = ApproximateTimeSynchronizer([botSub, slamtb], queue_size=1, slop=0.2)
    # ats.registerCallback(callPoseTemp)
    ats.registerCallback(callback)
    
    # print(trans)
    # print(rot)
    


if __name__ == '__main__':
    os.system("rm bot.kitti vicon_gt.kitti slam.kitti")
    # os.system("rm bot.kitti")
    rospy.init_node("main", anonymous=True)

    vicon_gt = open("vicon_gt.kitti", "ab")
    bot_odom = open("bot.kitti", "ab")
    slam = open("slam.kitti", "ab")

    slamTf = tf.TransformListener()
    
    loc_extract()
    
    rospy.spin()
