import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher("mtg_agent_bringup_node/agent_0/cmd_vel", Twist, queue_size=1)
rospy.init_node("8shape", anonymous=True)
rate = rospy.Rate(10)

def clock(now, t, dir):
    while rospy.Time.now() < now + rospy.Duration.from_sec(t):
        msg = Twist()
        msg.angular.z = dir * 1.2
        msg.linear.x = 0.24
        pub.publish(msg)
        rate.sleep()
    return

for i in range(2):
    now = rospy.Time.now()
    if i == 0:
        clock(now, 5, 1)
    else:
        clock(now, 5, -1)