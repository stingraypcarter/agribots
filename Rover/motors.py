import numpy as np
import math
import time
from enum import IntEnum

class DriveMode(IntEnum):
    DRIVE = 1
    BRAKE = 2
    COAST = 3

# pid_consts = np.array([300,2,2])
pid_consts = np.array([4,0,1]) #0.01
int_limit = 100

IN_forward = np.array([1,0])
IN_back = np.array([0,1])
IN_coast = np.array([1,1])
IN_brake = np.array([0,0])

motors_min_effort = 0
motors_max_effort = 100

# pid_e_consts = np.array([1,1,1])
# int_overflow_lim = 10
# enc_vel_error = np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0]])
# enc_vel_cur = np.array([1,2,3,4])
# enc_vel_desired = np.array([2,2,2,2])
# pid_e_t = time.time()


def motor_init():
    motors_active = True
    drive_mode = DriveMode.COAST
    # ang_vel_cur = np.zeros(4)
    ang_vel_desired = np.zeros(4)
    motor_error = np.zeros((3,4))
    pid_t = time.time()
    motor_efforts = np.zeros(4)
    motor_dir = np.ones(4)
    return (motors_active, drive_mode, ang_vel_desired, motor_error, pid_t, motor_efforts, motor_dir)

def integrate_control(error, last_error, last_sum_error, pid_dt):
    error_int = np.zeros(4)
    for i in range(4):
        if not (error[i] == 0 or last_error[i] == 0):
            if error[i]/abs(error[i]) != last_error[i]/abs(last_error[i]):
                # integral windup
                error_int[i] = 0
            else:
                error_int[i] = np.clip(last_sum_error[i] + (error[i] * pid_dt), - int_limit, int_limit)
        else:
            error_int[i] = np.clip(last_sum_error[i] + (error[i] * pid_dt), - int_limit, int_limit)
    return error_int


def motor_pid(ang_vel_cur, ang_vel_desired, motor_error, pid_t, motor_efforts, motor_dir):
    motor_efforts = np.abs(motor_efforts)

    if ang_vel_desired[0] > 0:
        motor_dir[0] = 1
    elif ang_vel_desired[0] < 0:
        motor_dir[0] = -1

    if ang_vel_desired[1] > 0:
        motor_dir[1] = 1
    elif ang_vel_desired[1] < 0:
        motor_dir[1] = -1

    motor_dir[2] = motor_dir[0]
    motor_dir[3] = motor_dir[1]

    # motor_dir[0]
    # motor_dir[0] = 1 if ang_vel_desired[0] >= 0 else -1
    # motor_dir[2] = motor_dir[0]
    # motor_dir[1] = 1 if ang_vel_desired[1] >= 0 else -1
    # motor_dir[3] = motor_dir[1]

    ang_vel_desired = np.abs(ang_vel_desired)
    ang_vel_cur = np.abs(ang_vel_cur)

    pid_t_cur = time.time()
    pid_dt = pid_t_cur - pid_t

    error = ang_vel_desired - ang_vel_cur
    # print('error:')
    # print(error)
    # print(ang_vel_desired)
    # print(ang_vel_cur)
    last_error = motor_error[0,:]
    last_sum_error = motor_error[1,:]

    error_int = integrate_control(error,last_error,last_sum_error, pid_dt)

    error_dot = (error - last_error) / pid_dt

    error_full = np.array([error, error_int, error_dot])
    efforts = np.clip((error_full.T @ pid_consts),-100,100)
    efforts = np.clip((motor_efforts + efforts),0,100) * motor_dir
    return (efforts, error_full, pid_t, motor_dir)

def control_drive(efforts,drive_mode = DriveMode.DRIVE):
    IN_write = np.zeros(4)
    PWM_write = np.zeros(4)

    if drive_mode == DriveMode.DRIVE:
        IN_write[0:2] = IN_forward if efforts[0] >= 0 else IN_back
        # motor_dir[0] = 1 if efforts[0] >= 0 else -1
        # motor_dir[2] = 1 if efforts[0] >= 0 else -1

        IN_write[2:4] = IN_forward if efforts[1] >= 0 else IN_back
        # motor_dir[1] = 1 if efforts[1] >= 0 else -1
        # motor_dir[3] = 1 if efforts[1] >= 0 else -1

        PWM_write = np.clip(np.rint(np.abs(efforts)),motors_min_effort,motors_max_effort)
    else:
        if(drive_mode == DriveMode.COAST):
            IN_write = IN_coast
        else:
            IN_write = IN_brake
        PWM_write = np.zeros(4)
    return ((IN_write, PWM_write))
