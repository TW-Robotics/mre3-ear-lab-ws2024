#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

class moveAbort:
    """Cancels move base goal in case teleop/cmd_vel is received """
    # Cancel move base goals: https://answers.ros.org/question/57772/how-can-i-cancel-a-navigation-command/
    def __init__(self):
        self.msg = GoalID()
        self.cancelpub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.cmd_velsub = rospy.Subscriber("teleop/cmd_vel", Twist, self.callback)
        self.stamp = rospy.Time.now()

    def callback(self, _):
        now = rospy.Time.now()
        # Check if it has been long enough since last publish
        #TODO: Also check if goal is active
        if (now - self.stamp).to_sec() > 5.0:
            self.cancelpub.publish(self.msg)
            self.stamp = now

if __name__ == "__main__":
    # Instantiate node and keep it alive
    rospy.init_node("goal_preemption")
    node = moveAbort()
    rospy.spin()
