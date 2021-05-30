#!/usr/bin/env python3

import sys, rospy
from geometry_msgs.msg import Twist, Accel
# create variable ros mags
twist = Twist()
accel = Accel()
# hz rate of this node
Hz_command = 50
# index list
linear_x = 0
angular_z = 1
# size of list
num_axis = 2
# define dict to memory variable to cal
value = {   'target_vel' : [0] * num_axis,      # target velocity from key_input.py
            'pre_target_vel' : [0] * num_axis,  # memory last target velocity
            'acc_max' : [0] * num_axis,         # max acceleration from key_input.py
            'vel_now' : [0] * num_axis,         # memory current velocity from this node cal
            'acc_now' : [0] * num_axis,         # memory current acceleration from this node cal
            'coef' : [[0] * 4] * num_axis,      # constant coefficients to find vel of each time
            'tau' : [0] * num_axis,             # count time since find coef finish
            'T' : [0] * num_axis}               # limit time of coef traj
# define formula variable with lambda to find velocity & acceleration of each time
vel_cal = lambda Coef, tau : Coef[0] + (Coef[1]*tau) + (Coef[2]*(tau**2)) + (Coef[3]*(tau**3))
acc_cal = lambda Coef, tau : Coef[1] + (2*Coef[2]*tau) + (3*Coef[3]*(tau**2))
# define formula variable with lambda to find coef
coef_cal = lambda vi, ai, vf, af, T : [ vi, 
                                        ai, 
                                        ((3*(vf-vi))/(T**2))-((af+(2*ai))/T), 
                                        -((2*(vf-vi))/(T**3))+((af+ai)/(T**2))]
# define formula to find max velocity to check limit accelation
def tau_acc_peek(Coef):
    if Coef[3]:
        return -Coef[2]/(3*Coef[3])
    else:
        return 0
def callback_acc_limit(data = Accel()):
    global value
    value['acc_max'][linear_x] = data.linear.x
    value['acc_max'][angular_z] = data.angular.z
# function callback when receive topic /vel_raw
def callback_vel_raw(data = Twist()):
    # import global variable
    global value, num_axis
    # define new target velocity
    value['target_vel'][linear_x] = data.linear.x
    value['target_vel'][angular_z] = data.angular.z

    for axis in range(num_axis):
        if value['target_vel'][axis] != value['pre_target_vel'][axis]:
            value['pre_target_vel'][axis] = value['target_vel'][axis]
            value['T'][axis] = 0.1
            """
                loop increase time(T) for check limit acceleration to find best coef
                    1. velocity target
                    2. max acceleration
            """
            exitt = True
            while exitt:
                exitt = False
                value['coef'][axis] = coef_cal( value['vel_now'][axis], 
                                                value['acc_now'][axis], 
                                                value['target_vel'][axis], 
                                                0, 
                                                value['T'][axis])
                tau = tau_acc_peek(value['coef'][axis])
                acc = acc_cal(value['coef'][axis], tau)
                if abs(acc) > value['acc_max'][axis]:
                    value['T'][axis] += 0.1
                    exitt = True
            value['tau'][axis] = 0

if __name__ == '__main__':
    # define node name of this python node
    rospy.init_node('analyze_vel', anonymous=True)
    # define variable with topic to publish
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub_acc = rospy.Publisher('/chk_acc', Accel, queue_size=10)
    # define variable with topic to subscribe from key_input.py
    rospy.Subscriber('/vel_raw', Twist, callback_vel_raw)
    rospy.Subscriber('/acc_limit', Accel, callback_acc_limit)
    # define work rate of this node
    r = rospy.Rate(Hz_command)
    while(not rospy.is_shutdown()):
        try:
            # loop find velocity & acceleration of each axis
            for axis in range(num_axis):
                # when tau < T
                if value['tau'][axis] < value['T'][axis]:
                    value['vel_now'][axis] = vel_cal(   value['coef'][axis], 
                                                        value['tau'][axis])
                    value['acc_now'][axis] = acc_cal(   value['coef'][axis], 
                                                        value['tau'][axis])
                # when tau >= T
                else:
                    value['vel_now'][axis] = value['target_vel'][axis]
                    value['acc_now'][axis] = 0
                value['tau'][axis] += (1 / Hz_command)
            # update velocity & acceleration in variable
            twist.linear.x = value['vel_now'][linear_x]
            twist.angular.z = value['vel_now'][angular_z]
            accel.linear.x = value['acc_now'][linear_x]
            accel.angular.x = value['acc_now'][angular_z]
            # pub vel & acc msgs to ros
            pub_vel.publish(twist)
            pub_acc.publish(accel)
            # wait until 1/Hzrate
            r.sleep()
        # define for exit with ros interrupt exception
        except rospy.ROSInterruptException:
            sys.exit()