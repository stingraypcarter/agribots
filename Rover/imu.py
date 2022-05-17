import numpy as np
import math
import time

# hard set from hub facing
start_deg = -65.5

def imu_init(pose):
    imu_data = np.zeros(9)
    pose_ie = np.zeros((3,3))
    error_axes = np.zeros((3,3))
    pose_ie_t = time.time()
    return(imu_data, pose_ie, error_axes, pose_ie_t)

def imu_mag_init():
    cur_heading = 0
    return cur_heading

def imu_mag(mag_data):
    mx, my, mz = mag_data
    deg = math.atan2(mx,my) * (180/math.pi)
    return deg - start_deg


def imu_estimate(imu_data, pose, pose_ie_t, error_axes):
    pose_ie_t_cur = time.time()
    pose_ie_dt = pose_ie_t_cur - pose_ie_t

    x_acc,y_acc,z_acc = imu_data[0],imu_data[2],imu_data[1]
    pitch_acc,roll_acc,theta_acc = imu_data[3], imu_data[5], imu_data[4]

    # mag_x, mag_y, mag_z = imu_data[6], imu_data[7], imu_data[8]
    # so far unused

    pose_ie_ddot = np.array([x_acc,y_acc,pitch_acc])
    pose_ie_dot = pose[1,:] + (pose_ie_ddot * pose_ie_dt)
    pose_ie = pose[0,:] + (pose_ie_dot * pose_ie_dt)

    pose_ie_full = np.array([pose_ie,pose_ie_dot,pose_ie_ddot])

    error_axes_ie_ddot = np.array([z_acc,pitch_acc,roll_acc])
    error_axes_ie_dot = error_axes[1,:] + (error_axes_ie_ddot * pose_ie_dt)
    error_axes_ie = error_axes[0,:] + (error_axes_ie_dot * pose_ie_dt)

    error_axes_full = np.array([error_axes_ie,error_axes_ie_dot,error_axes_ie_ddot])

    return(pose_ie_full,error_axes_full, pose_ie_t_cur)
