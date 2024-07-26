import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from math import pow, sqrt
import math
import numpy as np

if __name__ == "__main__":
    rospy.init_node('mavros_circle', anonymous=True)

    current_state = State()
    pose = PoseStamped()
    radius = 5

    def state_cb(msg):
        global current_state
        current_state = msg

    state_sub = rospy.Subscriber("/mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    # local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, position_cb)

    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()
            else:
                # "an equation of a circle!"
                rot = Rotation.from_euler('xyz', [0, 0, -90], degrees=True)
                quat = rot.as_quat()

                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]

        local_pos_pub.publish(pose)
        rospy.loginfo(quat[0])
        rospy.loginfo(quat[1])
        rospy.loginfo(quat[2])
        rospy.loginfo(quat[3])
        rospy.loginfo("-----")

        rate.sleep()