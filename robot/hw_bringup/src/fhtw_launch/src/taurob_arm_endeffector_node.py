#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import serial

arduino = serial.Serial("/dev/ArduinoEEF", 9600, timeout=0.1)


def callback_arduino_comm(data_sys_to_end):
    arduino.write((data_sys_to_end.data).encode())


if __name__ == "__main__":
    print("Main started")
    try:
        rospy.init_node("communication_endeffector", anonymous=True)
        pub = rospy.Publisher("arduino_comm_end_to_sys", String, queue_size=10)
        rospy.Subscriber("arduino_comm_sys_to_end", String, callback_arduino_comm)

        fix_rotation = rospy.get_param("FIX_ROTATION", False)

        if fix_rotation is True:
            arduino.write(("6-0;").encode())
        elif fix_rotation is False:
            arduino.write(("7-0;").encode())

        rate = rospy.Rate(200)

        print("Endeffector communication node started")

        while not rospy.is_shutdown():

            if arduino.in_waiting > 0:
                data_end_to_sys = arduino.readline().decode()

                if data_end_to_sys:

                    if data_end_to_sys[0] >= "0" and data_end_to_sys[0] <= "9":

                        pub.publish(data_end_to_sys)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
