import numpy as np
import math
import matplotlib.pyplot as plt
import random
from datetime import datetime

# Test locations must be obtained with RTK base configuration
hub_origin_ll = (42.274315,-71.8084567)
hub_origin_deg = 34.35103851903750 #deg

# wheel_base = 1
wheel_track = 1
wheel_radius = 1

def encoder_estimate(enc_vel, pose, pose_ee_dt):
    FL,FR,RL,RR = enc_vel[0,0],enc_vel[0,1],enc_vel[1,0],enc_vel[1,1]
    omega_l, omega_r = FL if abs(FL) < abs(RL) else RL, FR if abs(FR) < abs(RR) else RR
    v_l, v_r = omega_l * wheel_radius, omega_r * wheel_radius
    omega = (v_r - v_l)/wheel_track
    x, y, theta = pose[0,0], pose[0,1], pose[0,2]
    if(v_r == v_l):
        v = v_r
        R = float('inf')
        ICC = np.array([0,0])
        turn_ee = np.array([ICC[0],ICC[1],R])
        pose_ee = np.array([x+v*np.cos(theta)*pose_ee_dt,y+v*np.sin(theta)*pose_ee_dt,theta])
    elif(v_r == -v_l):
        v = v_r
        R = 0
        ICC = np.array([x,y])
        turn_ee = np.array([ICC[0],ICC[1],R])
        pose_ee = np.array([x,y,theta+(2*v*pose_ee_dt)/wheel_track])
    else:
        R = (wheel_track/2)*((v_l+v_r)/(v_r-v_l))
        ICC = np.array([x-R*np.sin(theta),y+R*np.cos(theta)])
        turn_ee = np.array([ICC[0],ICC[1],R])
        d_theta = omega * pose_ee_dt
        SO3_discrete =  np.array([[np.cos(d_theta),-np.sin(d_theta),0],[np.sin(d_theta),np.cos(d_theta),0],[0,0,1]])
        rel_ICC = np.array([x-ICC[0],y-ICC[1],theta])
        offset_ICC = np.array([ICC[0],ICC[1],d_theta])
        pose_ee = SO3_discrete @ rel_ICC + offset_ICC
    pose_ee_dot = (pose_ee - pose[0,:]) / pose_ee_dt
    pose_ee_ddot = (pose_ee_dot - pose[1,:]) / pose_ee_dt
    pose_ee_full = np.array([pose_ee,pose_ee_dot,pose_ee_ddot])
    return(pose_ee_full,turn_ee)


def imu_estimate(imu_acc, pose, pose_ie_dt, error_axes):
    x_acc,y_acc,z_acc = imu_acc[0,0],imu_acc[0,2],imu_acc[0,1]
    pitch_acc,roll_acc,theta_acc = imu_acc[1,0], imu_acc[1,2], imu_acc[1,1]

    pose_ie_ddot = np.array([x_acc,y_acc,pitch_acc])
    pose_ie_dot = pose[1,:] + (pose_ie_ddot * pose_ie_dt)
    pose_ie = pose[0,:] + (pose_ie_dot * pose_ie_dt)

    pose_ie_full = np.array([pose_ie,pose_ie_dot,pose_ie_ddot])

    error_axes_ie_ddot = np.array([z_acc,pitch_acc,roll_acc])
    error_axes_ie_dot = error_axes[1,:] + (error_axes_ie_ddot * pose_ie_dt)
    error_axes_ie = error_axes[0,:] + (error_axes_ie_dot * pose_ie_dt)

    error_axes_full = np.array([error_axes_ie,error_axes_ie_dot,error_axes_ie_ddot])
    return(poe_ie_full,error_axes_ie)

def ll2xy(ll,origin_ll,origin_deg,scalers):
    NE = ((ll[0]-origin_ll[0])*scalers[0],(ll[1]-origin_ll[1])*scalers[1])
    psi = deg2rad(-origin_deg)
    x = math.sin(psi)*NE[0] + math.cos(psi)*NE[1]
    y = math.cos(psi)*NE[0] - math.sin(psi)*NE[1]
    return (x,y)

def xy2ll(xy,origin_ll,origin_deg,scalers):
    psi = deg2rad(origin_deg)
    N = math.cos(psi)*xy[1]- math.sin(psi)*xy[1]
    E = math.sin(psi)*xy[1]+ math.cos(psi)*xy[1]
    return (origin_ll[0] + (N/scalers[0]),origin_ll[1] + (E/scalers[1]))

def deg2m(pos_gps):
    mu = pos_gps[0]*math.pi/180
    # constants from wikipedia https://en.wikipedia.org/wiki/Geographic_coordinate_system#Length_of_a_degree
    lat = 111132.92 - 559.82 * math.cos(2*mu) + 1.175 * math.cos(4*mu) -0.0023 * math.cos(6*mu)
    lon = 111412.84 * math.cos(mu) -93.5* math.cos(3*mu) + 0.118 * math.cos(5*mu)
    return (lat,lon)

def decode_GPS_RMC(str):
    data = str.split(',')
    header = data[0]
    str_UTC_time = data[1]
    str_time_valid = data[2]
    str_latitude = data[3]
    str_latitude_north = data[4]
    str_longitude = data[5]
    str_longitude_east = data[6]
    str_ground_speed = data[7]
    str_ground_course = data[8]
    str_UTC_date = data[9]

    # assuming UTC date is valid from error check bit
    str_datetime = str_UTC_time+'0000'+str_UTC_date
    datetime_obj = datetime.strptime(str_datetime,'%H%M%S.%f%d%m%y')

    latitude_north = str_latitude_north == 'N'
    longitude_east = str_longitude_east == 'E'

    latitude = float(str_latitude[0:2])+(float(str_latitude[2:])/60.0)
    if not latitude_north:
        latitude *= -1

    longitude = float(str_longitude[0:3])+(float(str_longitude[3:])/60.0)
    if not longitude_east:
        longitude *= -1
    GPS_pos = (latitude,longitude)

    return (datetime_obj,GPS_pos)


def gen_test_enc_vel(rot_vels,noise_amp):
    samples = len(rot_vels)
    enc_vel = np.zeros((samples,2,2))
    for i, s in enumerate(rot_vels):
        enc_vel[i,0,0] = s[0]+random.random()*noise_amp
        enc_vel[i,0,1] = s[1]+random.random()*noise_amp
        enc_vel[i,1,0] = s[0]+random.random()*noise_amp
        enc_vel[i,1,1] = s[1]+random.random()*noise_amp
    return enc_vel


def gen_test_diff_drive(pose_ee_dt,duration):
    samples = int(round(duration/pose_ee_dt))
    time = 0
    vel_set = []

    for i in range(samples):
        wL = np.cos(np.sin(time))
        wR = np.sin(np.cos(time))
        # wL = .1
        # wR = .15
        vel_set.append((wL,wR))
        time += pose_ee_dt
    return vel_set

pose_ee_dt_test = 0.01
duration = 400
pose_test = np.array([[0,0,math.pi/2],[0,0,0],[0,0,0]])


vel_test = gen_test_diff_drive(pose_ee_dt_test,duration)
rot_vel_set = gen_test_enc_vel(vel_test,0)

x_vals = np.array([pose_test[0,0]])
y_vals = np.array([pose_test[0,1]])

for i in range(rot_vel_set.shape[0]):
    rot_vels = rot_vel_set[i]
    pose_test = encoder_estimate(rot_vels,pose_test,pose_ee_dt_test)[0]
    x_vals = np.concatenate((x_vals, np.array([pose_test[0,0]])), axis=0)
    y_vals = np.concatenate((y_vals, np.array([pose_test[0,1]])), axis=0)


fig, axs = plt.subplots(1, 1)

axs.axis('equal')

axs.plot(x_vals, y_vals)
plt.show()

# to get decimal degrees take digits after first 2 and divide by 60
# an example message from the RTK is parsed, resulting value should
# be a position in meters
# out = decode_GPS_RMC('$GNRMC,233205.00,A,4216.35728,N,07148.56841,W,0.025,,210222,,,A*71')
# scalers = deg2m(hub_origin_ll)
# xy = ll2xy(out[1],hub_origin_ll,hub_origin_deg,scalers)
# print(xy)
