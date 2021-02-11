#! /usr/bin/env python

import rospy
from mercury_msgs.msg import motors

def check_for_collision(data):
    pub = rospy.Publisher("/mercury/motor_out", motors, queue_size=1)
    # TODO: some sort of better collision avoidance logic
    collision_condition = False
    if (collision_condition):
        stop = motors()
        stop.left = 0
        stop.right = 0
        pub.publish(stop)
    else:
        pub.publish(data)


def avoid_collision():
    rospy.init_node("mercury_collision_avoidance")
    rospy.loginfo("Collision Avoidance Node Started")
    rospy.Subscriber("/mercury/motor_in", motors, check_for_collision)

    rospy.spin()

if __name__ == "__main__":
    try:
        avoid_collision()
    except rospy.ROSInterruptException:
        pass