from controller import Robot, Motor, GPS, InertialUnit
import sys
sys.path.append('../../../../controllers_shared/python_based')
print("Path appended successfully")
try:
    from pid_controller import pid_velocity_fixed_height_controller
    print("Imported pid_velocity_fixed_height_controller successfully")
except ImportError as e:
    print(f"Error importing pid_velocity_fixed_height_controller: {e}")

# Helper function to compute the motor power based on the PID controller
def compute_motor_power(pid_controller, dt, target_x, target_y, target_z, gps, imu):
    # Current position and orientation
    x, y, z = gps.getValues()
    roll, pitch, yaw = imu.getRollPitchYaw()

    print(f"Current position: x={x}, y={y}, z={z}")
    print(f"Target position: x={target_x}, y={target_y}, z={target_z}")

    # Compute errors
    error_x = target_x - x
    error_y = target_y - y
    error_z = target_z - z

    print(f"Errors: error_x={error_x}, error_y={error_y}, error_z={error_z}")

    # Convert the target coordinates to velocities (adjust scaling factor as needed)
    scaling_factor = 1.74  # Adjust this value to tune responsiveness
    forward_desired = error_x * scaling_factor
    sideways_desired = error_y * scaling_factor
    height_diff_desired = error_z * scaling_factor
    yaw_desired = 0  # For simplicity, we're not adjusting yaw here

    print(f"Desired: forward={forward_desired}, sideways={sideways_desired}, height_diff={height_diff_desired}")

    # PID control
    try:
        motor_power = pid_controller.pid(dt, forward_desired, sideways_desired,
                                         yaw_desired, height_diff_desired,
                                         roll, pitch, yaw,
                                         z, 0, 0)
    except Exception as e:
        print(f"Error calling pid method: {e}")
        motor_power = [0, 0, 0, 0]

    print(f"Motor power: {motor_power}")

    # Adjust motor power for correct direction
    motor_power[0] = -motor_power[0]  # Motor 1
    motor_power[1] = motor_power[1]   # Motor 2
    motor_power[2] = -motor_power[2]  # Motor 3
    motor_power[3] = motor_power[3]   # Motor 4

    return motor_power

# Initialize Webots robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize motors
motors = []
for i in range(1, 5):
    motor = robot.getDevice(f"m{i}_motor")
    motor.setPosition(float('inf'))
    motor.setVelocity(0)
    motors.append(motor)

# Initialize sensors
gps = robot.getDevice("gps")
gps.enable(timestep)
imu = robot.getDevice("inertial_unit")
imu.enable(timestep)

# Define pick-up and drop-off coordinates
pick_up_coords = [1.0, 1.0, 1.0]  # [x, y, z]
drop_off_coords = [2.0, 2.0, 1.0]  # [x, y, z]

# Create an instance of the PID controller
try:
    pid_controller = pid_velocity_fixed_height_controller()
    print("PID controller instantiated successfully")
except Exception as e:
    print(f"Error instantiating pid_velocity_fixed_height_controller: {e}")

# Move to pick-up coordinates
target_coords = pick_up_coords
while robot.step(timestep) != -1:
    dt = robot.getTime() - timestep
    print(f"Time step: {dt}")
    try:
        motor_power = compute_motor_power(pid_controller, dt, target_coords[0], target_coords[1], target_coords[2], gps, imu)
        print(f"Setting motor power: {motor_power}")
        for i in range(4):
            motors[i].setVelocity(motor_power[i])
            print(f"Motor {i+1} velocity set to: {motor_power[i]}")
    except Exception as e:
        print(f"Error in compute_motor_power: {e}")

    # Check if reached pick-up coordinates
    x, y, z = gps.getValues()
    print(f"Current position: x={x}, y={y}, z={z}")
    if abs(x - target_coords[0]) < 0.1 and abs(y - target_coords[1]) < 0.1 and abs(z - target_coords[2]) < 0.1:
        print("Reached pick-up coordinates")
        break

# Move to drop-off coordinates
target_coords = drop_off_coords
while robot.step(timestep) != -1:
    dt = robot.getTime() - timestep
    print(f"Time step: {dt}")
    try:
        motor_power = compute_motor_power(pid_controller, dt, target_coords[0], target_coords[1], target_coords[2], gps, imu)
        print(f"Setting motor power: {motor_power}")
        for i in range(4):
            motors[i].setVelocity(motor_power[i])
            print(f"Motor {i+1} velocity set to: {motor_power[i]}")
    except Exception as e:
        print(f"Error in compute_motor_power: {e}")

    # Check if reached drop-off coordinates
    x, y, z = gps.getValues()
    print(f"Current position: x={x}, y={y}, z={z}")
    if abs(x - target_coords[0]) < 0.1 and abs(y - target_coords[1]) < 0.1 and abs(z - target_coords[2]) < 0.1:
        print("Reached drop-off coordinates")
        break



