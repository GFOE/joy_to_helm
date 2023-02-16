#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from project11_msgs.msg import Helm
from project11_msgs.msg import DifferentialDrive
from project11_msgs.msg import Heartbeat
from std_msgs.msg import String

helm_publisher = None
dd_publisher = None
piloting_mode_publisher = None
state = 'standby'
drive_mode = 'helm'
allow_differential_drive = False

throttle_axis = 1
slow_mode_axis = 2
rudder_axis = 3

manual_button = 0
autonomous_button = 2
standby_button = 1



def heartbeatCallback(msg):
    global state
    for kv in msg.values:
        if kv.key == 'piloting_mode':
            state = kv.value
    

def joystickCallback(msg):
    global state
    global drive_mode

    state_request = None
    if msg.buttons[manual_button]:
        state_request = 'manual'
    if msg.buttons[autonomous_button]:
        state_request = 'autonomous'
    if msg.buttons[standby_button]:
        state_request = 'standby'
    if msg.buttons[9]:
        drive_mode = 'helm'
        print ('drive_mode',drive_mode)
    if msg.buttons[10] and allow_differential_drive:
        drive_mode = 'differential'
        print ('drive_mode',drive_mode)
    if state_request is not None and state_request != state:
        piloting_mode_publisher.publish('piloting_mode '+state_request)
        state = state_request
    
    if state == 'manual':
        if drive_mode == 'helm':
            limit_factor = 0.35
            if msg.axes[slow_mode_axis] < 0:
                limit_factor = 1.0
            helm = Helm()
            helm.header.stamp = rospy.Time.now()
            helm.throttle = msg.axes[throttle_axis]*limit_factor
            helm.rudder = -msg.axes[rudder_axis]
            helm_publisher.publish(helm)
        if drive_mode == 'differential':
            d = DifferentialDrive()
            d.header.stamp = rospy.Time.now()
            d.left_thrust = msg.axes[1]
            d.right_thrust = msg.axes[4]
            dd_publisher.publish(d)
    
if __name__ == '__main__':
    rospy.init_node('joy_to_helm')
    allow_differential_drive = rospy.get_param('allow_differential_drive', False)
    print ('allow_differential_drive:',allow_differential_drive)

    throttle_axis = rospy.get_param('~throttle_axis', throttle_axis)
    slow_mode_axis = rospy.get_param('~slow_mode_axis', slow_mode_axis)
    rudder_axis = rospy.get_param('~rudder_axis', rudder_axis)

    manual_button = rospy.get_param('~manual_button', manual_button)
    autonomous_button = rospy.get_param('~autonomous_button', autonomous_button)
    standby_button = rospy.get_param('~standby_button', standby_button)

    print('throttle_axis', throttle_axis)
    print('rudder_axis', rudder_axis)
    print('slow_mode_axis', slow_mode_axis)
    print('manual_button', manual_button)
    print('autonomous_button', autonomous_button)
    print('standby_button', standby_button)
    
    helm_publisher = rospy.Publisher('helm', Helm, queue_size=10)
    if allow_differential_drive:
      dd_publisher = rospy.Publisher('differential_drive', DifferentialDrive, queue_size=10)
    
    piloting_mode_publisher = rospy.Publisher('project11/send_command', String, queue_size=10)
    joy_subscriber = rospy.Subscriber('joy', Joy, joystickCallback)
    heartbeat_subscriber = rospy.Subscriber('project11/heartbeat', Heartbeat, heartbeatCallback)
    rospy.spin()
    
