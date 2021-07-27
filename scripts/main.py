#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
import math


angular_velocity = []
angular_velocity_covariance = []

linear_acceleration = []
linear_acceleration_covariance = []

magnetic_field = []
magnetic_field_covariance = []
Pk = np.zeros(4)
q =  np.array([[1.0],[0.0],[0.0],[0.0]])
start = 0.0
last = 0.0
time_flag = 0
g = 9.8

call_time = 0

#define error
linear_error = np.array([-0.168398173415,0.253387766867,0.0])
error_linear = np.array([0.0,0.0,0.0])
min_mag = np.array([0.0,0.0,0.0])
max_mag = np.array([0.0000000000001,0.0000000000001,0.0000000000001])
error_mag_min = np.array([-2.0356965065e-05,-5.12641727924e-05,-4.92941975594e-05])
error_mag_max = np.array([7.56806075573e-05,4.70717430115e-05,5.16282021999e-05])

#ros
pose_data = PoseStamped()

def normalize_mag(data):
    data[0] = (data[0] -error_mag_min[0])/ (error_mag_max[0]-error_mag_min[0])*2-1
    data[1] = (data[1] -error_mag_min[1])/ (error_mag_max[1]-error_mag_min[1])*2-1
    data[2] = (data[2] -error_mag_min[2])/ (error_mag_max[2]-error_mag_min[2])*2-1
    return data

def gyro_callback(data):
    #update
    global angular_velocity,angular_velocity_covariance,linear_acceleration,linear_acceleration_covariance
    angular_velocity = np.array([data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z])
    angular_velocity_covariance = np.array([[data.angular_velocity_covariance[0],data.angular_velocity_covariance[1],data.angular_velocity_covariance[2]],
                                            [data.angular_velocity_covariance[3],data.angular_velocity_covariance[4],data.angular_velocity_covariance[5]],
                                            [data.angular_velocity_covariance[6],data.angular_velocity_covariance[7],data.angular_velocity_covariance[8]]])
 
    linear_acceleration = np.array([data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z])
    linear_acceleration_covariance = np.array([[data.linear_acceleration_covariance[0],data.linear_acceleration_covariance[1],data.linear_acceleration_covariance[2]],
                                            [data.linear_acceleration_covariance[3],data.linear_acceleration_covariance[4],data.linear_acceleration_covariance[5]],
                                            [data.linear_acceleration_covariance[6],data.linear_acceleration_covariance[7],data.linear_acceleration_covariance[8]]])    
    '''
    print(linear_acceleration)
    #calculate average error 
    global error_linear , call_time
    call_time = call_time + 1
    error_linear[0] = linear_acceleration[0] + error_linear[0]
    error_linear[1] = linear_acceleration[1] + error_linear[1]
    error_linear[2] = linear_acceleration[2] + error_linear[2]
    print('error:')  
    print(float(error_linear[0])/call_time)
    print(float(error_linear[1])/call_time)
    print(float(error_linear[2])/call_time)
    print('---')                                    
    linear_acceleration = linear_acceleration - linear_error
    '''

def mag_callback(data):

    global time_flag,start,last
    global angular_velocity,angular_velocity_covariance,linear_acceleration,linear_acceleration_covariance,magnetic_field,magnetic_field_covariance
    global Pk,q

    if time_flag == 0:
        start = time.time()
        last = start
        time_flag = 1

    else:
        now_time = time.time()
        delta_time = now_time - last
        last = now_time
        #update
        
        magnetic_field = np.array([data.magnetic_field.x,data.magnetic_field.y,data.magnetic_field.z])
        magnetic_field_covariance = np.array([[data.magnetic_field_covariance[0],data.magnetic_field_covariance[1],data.magnetic_field_covariance[2]],
                                                [data.magnetic_field_covariance[3],data.magnetic_field_covariance[4],data.magnetic_field_covariance[5]],
                                                [data.magnetic_field_covariance[6],data.magnetic_field_covariance[7],data.magnetic_field_covariance[8]]])                                        

        '''
        print('mag_error:')  
        if min_mag[0] > data.magnetic_field.x :
            min_mag[0] = data.magnetic_field.x
        if min_mag[1] > data.magnetic_field.y :
            min_mag[1] = data.magnetic_field.y
        if min_mag[2] > data.magnetic_field.z :
            min_mag[2] = data.magnetic_field.z
        if max_mag[0] < data.magnetic_field.x :
            max_mag[0] = data.magnetic_field.x
        if max_mag[1] < data.magnetic_field.y :
            max_mag[1] = data.magnetic_field.y
        if max_mag[2] < data.magnetic_field.z :
            max_mag[2] = data.magnetic_field.z

        print(float(min_mag[0]))
        print(float(min_mag[1]))
        print(float(min_mag[2]))
        print(float(max_mag[0]))
        print(float(max_mag[1]))
        print(float(max_mag[2]))
        '''
        print('---')   
        print(magnetic_field)
        magnetic_field = normalize_mag(magnetic_field)
        print(magnetic_field)

        qc1 = np.array([[0.0],[0.0],[0.0],[0.0]])
        qc2 = np.array([[0.0],[0.0],[0.0],[0.0]])
        qk1 = np.array([[0.0],[0.0],[0.0],[0.0]])
    
        #param
        Vk1 = np.eye(3)
        Vk2 = np.eye(3)
        Q = 1e-7*np.eye(4,4)
        #Q = np.zeros((4,4))
        I4 = np.eye(4)
        magnetic_field_covariance = 1e-06*np.eye(3)
        linear_acceleration_covariance = 1e-05*np.eye(3)
        Rk1 = linear_acceleration_covariance
        Rk2 = magnetic_field_covariance



        #print(q)
        #update angular velocity using gyro 
        wx = angular_velocity[0]
        wy = angular_velocity[1]
        wz = angular_velocity[2]
        Omega = np.array([[0 ,  -wx,  -wy,  -wz],
                        [wx,    0,   wz,  -wy],
                        [wy,  -wz,    0,   wx],
                        [wz,   wy,  -wx,    0]])
        A = 0.5 * Omega # A is time-varing matrix
        Ak = I4 + A*float(delta_time)
        q = np.matmul(Ak,q)
        Pk = Ak*Pk*Ak.transpose() + Q

        #update angular velocity using accelerometer
        h1 = g * np.array([[2*(q[1,0]*q[3,0]-q[0,0]*q[2,0])],
                        [2*(q[0,0]*q[1,0]+q[2,0]*q[3,0])],
                        [q[0,0]*q[0,0]+q[3,0]*q[3,0]-(q[1,0]*q[1,0]+q[2,0]*q[2,0])]])
        Hk1 = np.array([[-2*q[2,0], 2*q[3,0], -2*q[0,0], 2*q[1,0]],
                        [2*q[1,0], 2*q[0,0], 2*q[3,0], 2*q[2,0]],
                        [2*q[0,0], -2*q[1,0], -2*q[2,0], 2*q[3,0]]])
        Hk1_t = Hk1.transpose()
        Vk1_t = Vk1.transpose()
        A = np.linalg.inv(np.matmul(np.matmul(Hk1,Pk),Hk1_t)+Vk1*Rk1*Vk1_t)
        Kk1 = np.matmul(np.matmul(Pk,Hk1_t),A)

        zk1 = np.array([[linear_acceleration[0]],[linear_acceleration[1]],[linear_acceleration[2]]])
        qc1 = np.matmul(Kk1,(zk1-h1))
        qc1[3,0] = 0
        qk1 = q+qc1
        Pk1 = np.matmul(I4 - np.matmul(Kk1,Hk1),Pk)

        #update angular velocity using magnetic field
        h2 = np.array([[2*(q[1,0]*q[2,0]+q[0,0]*q[3,0])],
                        [q[0,0]*q[0,0]-q[1,0]*q[1,0]-q[2,0]*q[2,0]-q[3,0]*q[3,0]],
                        [2*(q[2,0]*q[3,0]-q[0,0]*q[(1,0)])]])
        Hk2 = np.array([[2*q[3,0], 2*q[2,0], 2*q[1,0], 2*q[0,0]],
                        [2*q[0,0], -2*q[1,0], -2*q[2,0], -2*q[3,0]],
                        [-2*q[1,0], -2*q[0,0], 2*q[3,0], 2*q[2,0]]])

        Hk2_t = Hk2.transpose()
        Vk2_t = Vk2.transpose()
        B = np.linalg.inv(np.matmul(np.matmul(Hk2,Pk),Hk2_t)+Vk2*Rk2*Vk2_t)
        Kk2 = np.matmul(np.matmul(Pk,Hk2_t),B)

        zk2 = np.array([[magnetic_field[0]],[magnetic_field[1]],[magnetic_field[2]]])
        qc2 = np.matmul(Kk2,(zk2-h2))
        qc2[1,0] = 0
        qc2[2,0] = 0
        q=qk1+qc2

        #update q
        normq = math.sqrt(q[0,0]*q[0,0]+q[1,0]*q[1,0]+q[2,0]*q[2,0]+q[3,0]*q[3,0])
        q = np.array([[q[0,0]/float(normq)],[q[1,0]/float(normq)],[q[2,0]/float(normq)],[q[3,0]/float(normq)]])

        #calculate posterior
        Pk = np.matmul(I4 - np.matmul(Kk2,Hk2),Pk1)
        pose_data.pose.orientation.x = q[1,0]
        pose_data.pose.orientation.y = q[2,0]
        pose_data.pose.orientation.z = q[3,0]
        pose_data.pose.orientation.w = q[0,0]
        pose_data.header.frame_id = "map"
        pose_state.publish(pose_data)

        #print(q)


if __name__ == '__main__':
    rospy.init_node('double_stage_kalman')
    rospy.Subscriber("/mavros/imu/data_raw", Imu, gyro_callback) 
    rospy.Subscriber("/mavros/imu/mag", MagneticField, mag_callback) 
    pose_state = rospy.Publisher("Pose", PoseStamped, queue_size=1)

    rospy.spin()
