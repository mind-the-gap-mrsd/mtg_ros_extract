import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher("mtg_agent_bringup_node/agent_0/cmd_vel", Twist, queue_size=1)
rospy.init_node("Tshape", anonymous=True)
rate = rospy.Rate(10)

def straight(now, t):
    while rospy.Time.now() < now + rospy.Duration.from_sec(t):
        msg = Twist()
        msg.linear.x = 0.16
        pub.publish(msg)
        rate.sleep()
    return

def turn(now, t, dir):
    while rospy.Time.now() < now + rospy.Duration.from_sec(t):
        msg = Twist()
        msg.angular.z = dir * 0.8
        pub.publish(msg)
        rate.sleep()
    return


for i in range(10):
    now = rospy.Time.now()
    if i in [0,2,4,6,8]:
        if i == 4:
            straight(now, 10)
        else:
            straight(now, 5)
    else:
        if i in [1,7]:
            turn(now, 2, 1)
        else:
            turn(now, 4, -1)