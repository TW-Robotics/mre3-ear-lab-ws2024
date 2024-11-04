#! /usr/bin/env python

import sys
import signal
import ast # ast instead of json, see: https://stackoverflow.com/questions/4162642/single-vs-double-quotes-in-json
import rospy
# Utilised message types must be imported here
from geometry_msgs.msg import Twist, PoseStamped

class Mux:
    """Non timer-based mux instance that is configured from config dict (see config.json in ros package)
    """
    def __init__(self, configDict):

        # Store config and utility variables
        dtype = eval(configDict["dType"])                                   # Published/Subscribed message type
        self.lastActiveTimestamps = [ None for _ in configDict["topics"] ]  # Tracks timestamp of each subscriber's last callback
        self.latestInput = len(self.lastActiveTimestamps)                   # Tracks latest input priority, initialised with lowest priority
        self.cooldownTime = float(configDict["cooldownTime"])               # Time since last message for a channel to be considered inactive
        self.config = configDict

        # Instantiate subscribers and publisher
        # Add arguments to callback: https://answers.ros.org/question/231492/passing-arguments-to-callback-in-python/
        self.pub = rospy.Publisher(configDict["outTopic"], dtype, queue_size=1)
        self.subscribers = [ 
            rospy.Subscriber(topic, dtype, self.callback, (idx, topic)) 
            for idx, topic in enumerate(configDict["topics"]) 
        ] # Subscribers in descending priority

    def callback(self, data, args):
        idx, topic = args
        latestInput = self.latestInput
        now = rospy.Time.now()
        # Track that this callback has been active
        self.lastActiveTimestamps[idx] = now
        # Proceed under two conditions: latest input more important than we are (lower index = higher priority)
        # Or cooldown is shorter than the time passed since the last update
        if idx <= latestInput or self.cooldownTime <= (now - self.lastActiveTimestamps[latestInput]).to_sec():
            # Forward message on output and track that we have sent a message in self.latestInput
            self.latestInput = idx
            self.pub.publish(data)

def signal_handler(signal, frame):
    print('\n')
    sys.exit(0)

if __name__ == '__main__':
    configDict = ast.literal_eval(sys.argv[1])
    # Load config and init node and mux
    rospy.init_node(configDict["name"])
    mux = Mux(configDict)

    # Register sigint
    signal.signal(signal.SIGTERM, signal_handler)

    # Keep node alive and let callbacks handle the rest
    rospy.spin()
