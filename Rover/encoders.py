import time
import numpy as np
import math

# for 1x4 arrays of type encoder position or velocity
# Front Left, Front Right, Rear Left, Rear Right

# number of data points of tick values and time for a moving average velocity
enc_history_size = 15
# experimentally determined for each wheel (rotation/tick)
# exactly 800 ticks for 20 rotations -> 0.025
enc_rot_per_tick =  np.array([0.025,0.025,0.025,0.025])

# horizontal distance between center of wheels (m)
wheel_track = 1.04 #meters
# wheel radius (m)
wheel_radius = .275 #meters

# enc_raw:
    # encoder tick binary (1x4)
# enc_vel:
    # encoder velocity (1x4) (rad/s)
# enc_pos_data:
    # encoder tick binary (1x4)
    # current iteration (int)
    # current time (s)
# encoder history:
    # encoder delta positions (history size x 4)
    # encoder delta times (1x4)

def encoder_init(pose):
    enc_pos_data = (np.zeros(4),-1,time.time())
    enc_history = (np.zeros((enc_history_size,4)),np.zeros(enc_history_size))
    enc_vel = np.zeros(4)
    turn_ee = np.zeros(3)
    pose_ee_t = time.time()
    return (enc_vel, enc_pos_data, enc_history, pose, turn_ee, pose_ee_t)

def encoder_measure(enc_raw, enc_pos_data, enc_history, motor_dir):
    enc_pos, enc_m_i, enc_m_t = enc_pos_data
    enc_d_history, enc_dt_history = enc_history

    enc_m_i_cur = (enc_m_i+1) % enc_history_size
    enc_m_t_cur = time.time()

    enc_delta = np.abs(enc_raw - enc_pos)

    enc_d_history[enc_m_i_cur,:] = enc_delta
    enc_dt_history[enc_m_i_cur] = enc_m_t_cur - enc_m_t

    enc_tick_per_sec = np.sum(enc_d_history,axis = 0) / np.sum(enc_dt_history)

    enc_vel = (enc_tick_per_sec * enc_rot_per_tick * 2 * math.pi) * motor_dir

    return (enc_vel, (enc_raw, enc_m_i_cur, enc_m_t_cur), (enc_d_history, enc_dt_history))

# pose and pose_ee: (3x3)
    # x,y,theta; d/dt(x,y,theta), d^2/dt(x,y,theta) (m,m,rads)
# turn_ee: (1x3)
    # ICC_x, ICC_y, turning radius (m,m,m)
# pose_ee_t:
    # time of encoder estimate update

def encoder_estimate(enc_vel, pose, pose_ee_t):
    pose_ee_t_cur = time.time()
    pose_ee_dt = pose_ee_t_cur - pose_ee_t
    FL,FR,RL,RR = enc_vel[0],enc_vel[1],enc_vel[2],enc_vel[3]
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

    return(pose_ee_full, turn_ee, pose_ee_t_cur)
