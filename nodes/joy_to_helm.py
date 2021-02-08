#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from marine_msgs.msg import Helm
from marine_msgs.msg import DifferentialDrive
from marine_msgs.msg import Heartbeat
from std_msgs.msg import String

helm_publisher = None
dd_publisher = None
piloting_mode_publisher = None
state = 'standby'
drive_mode = 'helm'
allow_differential_drive = False

def heartbeatCallback(msg):
    global state
    for kv in msg.values:
        if kv.key == 'piloting_mode':
            state = kv.value
    

def joystickCallback(msg):
    global state
    global drive_mode

    state_request = None
    if msg.buttons[0]:
        state_request = 'manual'
    if msg.buttons[1]:
        state_request = 'autonomous'
    if msg.buttons[2]:
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
            helm = Helm()
            helm.header.stamp = rospy.Time.now()
            helm.throttle = msg.axes[1]
            helm.rudder = -msg.axes[3]
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
    
    helm_publisher = rospy.Publisher('/helm', Helm, queue_size=10)
    dd_publisher = rospy.Publisher('/differential_drive', DifferentialDrive, queue_size=10)
    
    piloting_mode_publisher = rospy.Publisher('/send_command', String, queue_size=10)
    joy_subscriber = rospy.Subscriber('/joy', Joy, joystickCallback)
    heartbeat_subscriber = rospy.Subscriber('/heartbeat', Heartbeat, heartbeatCallback)
    rospy.spin()
    
