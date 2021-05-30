#!/usr/bin/env python3

import sys
from pynput.keyboard import Key, KeyCode, Listener
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Accel
# create variable ros mags
twist = Twist()
accel = Accel()
# hz rate of this node
Hz_command = 50
# set default of linear & angular velocity
linear_vel = 2
angular_vel = 2
# define key to control direction
'''
    'key' = key input from pynput.keyboard.Key or pynput.keyboard.KeyCode
    'command' = result direction of each 'key' in numpy array (easy to calculate)
    'press' = state of each 'key' in numpy array (easy to calculate)
        0 = release
        1 = press
'''
control_key = { 'linear_vel':{  'key':[Key.up, Key.down], 
                                'command':np.array((1, -1)), 
                                'press':np.array((0, 0))}, 
                'angular_vel':{ 'key':[Key.left, Key.right], 
                                'command':np.array((1, -1)), 
                                'press':np.array((0, 0))}}
# define key to setting velocity & acceleration 
'''
    index of setting_vel
    0 : linear_vel
    1 : linear_acc
    2 : angular_vel
    3 : angular_acc
'''
setting_vel = { KeyCode.from_char('q') : [1.1,  1,  1,  1], 
                KeyCode.from_char('z') : [0.9,  1,  1,  1], 
                KeyCode.from_char('w') : [1,    1.1,1,  1], 
                KeyCode.from_char('x') : [1,    0.9,1,  1], 
                KeyCode.from_char('e') : [1,    1,  1.1,1], 
                KeyCode.from_char('c') : [1,    1,  0.9,1], 
                KeyCode.from_char('r') : [1,    1,  1,  1.1], 
                KeyCode.from_char('v') : [1,    1,  1,  0.9]}
# define function on_press
def on_press(key):
    # check control direction key
    for vel in ['linear_vel', 'angular_vel']:
        if key in control_key[vel]['key']:
            control_key[vel]['press'][control_key[vel]['key'].index(key)] = 1
# define function on_release
def on_release(key):
    # import global variable
    global linear_vel, angular_vel, accel
    # check control direction key
    for vel in ['linear_vel', 'angular_vel']:
        if key in control_key[vel]['key']:
            control_key[vel]['press'][control_key[vel]['key'].index(key)] = 0
    # check setting key
    if key in setting_vel:
        linear_vel *= setting_vel[key][0]
        accel.linear.x *= setting_vel[key][1]
        angular_vel *= setting_vel[key][2]
        accel.angular.z *= setting_vel[key][3]
        print('target linear vel: ' + str(linear_vel) + '\ntarget angular vel: ' + str(angular_vel) + 
        '\nmax linear acc: ' + str(accel.linear.x) + '\nmax angular acc: ' + str(accel.angular.z))
    # check esc key to exit
    if key == Key.esc:
        return False
# create listener function of keyboard when on press and on release
listener = Listener(on_press = on_press, on_release = on_release)

if __name__ == '__main__':
    # define node name of this python node
    rospy.init_node('key_input', anonymous=True)
    # define variable with topic to publish
    # vel topic from arg
    pub_vel = rospy.Publisher(sys.argv[1], Twist, queue_size=10)
    pub_acc = rospy.Publisher('/acc_limit', Accel, queue_size=10)
    # define work rate of this node 
    r = rospy.Rate(Hz_command)
    # run listener function
    listener.start()
    # define default acceleration
    accel.linear.x = 10
    accel.angular.z = 10
    while not rospy.is_shutdown():
        try:
            # cal velocity with mean of control direction key
            # linear vel cal
            num = control_key['linear_vel']['press'].sum()
            if num:
                linear = (control_key['linear_vel']['command'] * control_key['linear_vel']['press']).sum() / num
            else:
                linear = 0
            twist.linear.x = linear * linear_vel
            # angular vel cal
            num = control_key['angular_vel']['press'].sum()
            if num:
                angular = (control_key['angular_vel']['command'] * control_key['angular_vel']['press']).sum() / num
            else:
                angular = 0
            twist.angular.z = angular * angular_vel
            # pub vel & acc msgs to ros
            pub_vel.publish(twist)
            pub_acc.publish(accel)
            # wait until 1/Hzrate
            r.sleep()
        # define for exit with ros interrupt exception
        except rospy.ROSInterruptException:
            sys.exit()
        