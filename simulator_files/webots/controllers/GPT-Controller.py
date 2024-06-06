from controller import Robot, Motor, InertialUnit, GPS, Gyro, Keyboard, Camera, DistanceSensor
from math import cos, sin
import sys
sys.path.append('../../../../controllers_shared/python_based')
from pid_controller import pid_velocity_fixed_height_controller

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.util import SafeAsyncTask
from cflib.positioning.motion_commander import MotionCommander
import time

FLYING_ATTITUDE = 1

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Initialize motors
    m1_motor = robot.getDevice("m1_motor")
    m1_motor.setPosition(float('inf'))
    m1_motor.setVelocity(-1)
    m2_motor = robot.getDevice("m2_motor")
    m2_motor.setPosition(float('inf'))
    m2_motor.setVelocity(1)
    m3_motor = robot.getDevice("m3_motor")
    m3_motor.setPosition(float('inf'))
    m3_motor.setVelocity(-1)
    m4_motor = robot.getDevice("m4_motor")
    m4_motor.setPosition(float('inf'))
    m4_motor.setVelocity(1)

    # Initialize Sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    camera = robot.getDevice("camera")
    camera.enable(timestep)
    range_front = robot.getDevice("range_front")
    range_front.enable(timestep)
    range_left = robot.getDevice("range_left")
    range_left.enable(timestep)
    range_back = robot.getDevice("range_back")
    range_back.enable(timestep)
    range_right = robot.getDevice("range_right")
    range_right.enable(timestep)

    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)

    height_desired = 0.5
    past_time = robot.getTime()
    past_x_global = gps.getValues()[0]
    past_y_global = gps.getValues()[1]

    # Connect to Crazyflie
    uri = 'radio://0/80/250K'
    scf = SyncCrazyflie(uri, cf=Crazyflie())

    with scf:
        mc = MotionCommander(scf)
        
        while robot.step(timestep) != -1:
            dt = robot.getTime() - past_time
            roll = imu.getRollPitchYaw()[0]
            pitch = imu.getRollPitchYaw()[1]
            yaw = imu.getRollPitchYaw()[2]
            altitude = gps.getValues()[2]
            yaw_rate = gyro.getValues()[2]

            x_global = gps.getValues()[0]
            v_x_global = (x_global - past_x_global)/dt
            y_global = gps.getValues()[1]
            v_y_global = (y_global - past_y_global)/dt

            cosyaw = cos(yaw)
            sinyaw = sin(yaw)
            v_x = v_x_global * cosyaw + v_y_global * sinyaw
            v_y = - v_x_global * sinyaw + v_y_global * cosyaw

            desired_state = [0, 0, 0, 0]
            forward_desired = 0
            sideways_desired = 0
            yaw_desired = 0
            height_diff_desired = 0

            key = keyboard.getKey()
            while key > 0:
                if key == Keyboard.UP:
                    forward_desired += 0.5
                elif key == Keyboard.DOWN:
                    forward_desired -= 0.5
                elif key == Keyboard.RIGHT:
                    sideways_desired -= 0.5
                elif key == Keyboard.LEFT:
                    sideways_desired += 0.5
                elif key == ord('Q'):
                    yaw_desired = +1
                elif key == ord('E'):
                    yaw_desired = -1
                elif key == ord('W'):
                    height_diff_desired = 0.1
                elif key == ord('S'):
                    height_diff_desired = -0.1

                key = keyboard.getKey()

            height_desired += height_diff_desired * dt

            # Example: Move to specific coordinates
            mc.move_distance(forward_desired, sideways_desired, height_desired)

            motor_power = PID_CF.pid(dt, forward_desired, sideways_desired, yaw_desired, height_desired, roll, pitch, yaw_rate, altitude, v_x, v_y)
            
            m1_motor.setVelocity(-motor_power[0])
            m2_motor.setVelocity(motor_power[1])
            m3_motor.setVelocity(-motor_power[2])
            m4_motor.setVelocity(motor_power[3])

            past_time = robot.getTime()
            past_x_global = x_global
            past_y_global = y_global
