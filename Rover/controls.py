import numpy as np
import math
import time
# import RPi.GPIO as GPIO
from enum import IntEnum
# PID
# fl fr rl rr

class DriveMode(IntEnum):
    DRIVE = 1
    BRAKE = 2
    COAST = 3

pwm_freq = 100
motor_pwm_pins = np.array([1,2,3,4]) #for pwm
motor_dir_pins = np.array([9,10,11,12])
encoder_pins = np.array([5,6,7,8])

pid_e_consts = np.array([1,1,1])
int_overflow_lim = 10
enc_vel_error = np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0]])
enc_vel_cur = np.array([1,2,3,4])
enc_vel_desired = np.array([2,2,2,2])
pid_e_t = time.time()

def encoder_measure(enc_pos,enc_delta_history,iter):
    enc_pos_cur = [GPIO.input(enc_FL),GPIO.input(enc_FR),GPIO.input(enc_RL),GPIO.input(enc_RR)]
    enc_delta_cur = np.abs(enc_pos_cur - enc_pos)

def encoder_pid(enc_vel_cur,enc_vel_desired,enc_vel_error,pid_e_t,pid_e_consts,int_overflow_lim):
    pid_e_t_cur = time.time()
    pid_e_dt = pid_e_t_cur - pid_e_t

    error = enc_vel_desired - enc_vel_cur
    error_int = np.clip(enc_vel_error[1,:] + (error * pid_e_dt),-int_overflow_lim,int_overflow_lim)
    error_dot = (error - enc_vel_error[0,:])/pid_e_dt

    error_full = np.array([error,error_int,error_dot])

    efforts = error_full.T @ pid_e_consts
    return (efforts,error_full, pid_e_t_cur)

def sensor_actuator_init():
    # set mode
    GPIO.setmode(GPIO.BOARD)
    # motor pins
    GPIO.setup(ENA1, GPIO.OUT)
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)


def control_drive(efforts,drive_mode = DriveMode.DRIVE):
    if drive_mode == DriveMode.DRIVE:
        if(efforts[0] >= 0):
            GPIO.output(motor_dir_pins[0], GPIO.HIGH)
            GPIO.output(motor_dir_pins[1], GPIO.LOW)
        else:
            GPIO.output(motor_dir_pins[0], GPIO.LOW)
            GPIO.output(motor_dir_pins[1], GPIO.HIGH)

        if(efforts[1] >= 0):
            GPIO.output(motor_dir_pins[2], GPIO.HIGH)
            GPIO.output(motor_dir_pins[3], GPIO.LOW)
        else:
            GPIO.output(motor_dir_pins[2], GPIO.LOW)
            GPIO.output(motor_dir_pins[3], GPIO.HIGH)

        for i in range(4):
            PWM_FL = GPIO.PWM(motor_pwm_pins[i],PWM_FREQ)
            PWM_FL.start(abs(efforts[i]))
    else:
        if(drive_mode == DriveMode.COAST):
            set_mode = GPIO.LOW
        else:
            set_mode = GPIO.HIGH

        for i in range(4):
            GPIO.output(motor_dir_pins[i], set_mode)
            PWM_FL = GPIO.PWM(motor_pwm_pins[i],PWM_FREQ)
            PWM_FL.start(0)

together = encoder_pid(enc_vel_cur,enc_vel_desired,enc_vel_error,pid_e_t,pid_e_consts,int_overflow_lim)

control_drive(efforts)
