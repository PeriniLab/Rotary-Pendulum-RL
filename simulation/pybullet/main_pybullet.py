import pybullet as p
import os
import time
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([3.1415/2,0,0])
dir = os.path.abspath(os.path.dirname(__file__))
robot_urdf = "Rotary_Pendulum_URDF.urdf"
dir = os.path.join(dir,'simulation/urdf')
robot_urdf=os.path.join(dir,robot_urdf)
robotId = p.loadURDF(robot_urdf,cubeStartPos, cubeStartOrientation, 
                   # useMaximalCoordinates=1, ## New feature in Pybullet
                   flags=p.URDF_USE_INERTIA_FROM_FILE,
                   useFixedBase=True
                   )

# Find the joint index
motor_joint_index = [p.getJointInfo(robotId, i)[1] for i in range(p.getNumJoints(robotId))].index(b'Revolute_3')
bar_joint_index = [p.getJointInfo(robotId, i)[1] for i in range(p.getNumJoints(robotId))].index(b'Revolute_5')

# REAL ROBOT PARAMETERS
steps_per_rev = 3200
max_speed_steps_per_sec = 4000.0
# Calculate radians per step
radians_per_step = (2 * np.pi) / steps_per_rev
# Calculate max speed in radians per second [rad/s]
max_motor_speed = max_speed_steps_per_sec * radians_per_step 
# Admissible motor angle range [deg]
motor_angle_range = [-150, 150]

# URDF PARAMETERS
motor_compensation_angle = 0.400 # [rad]
bar_compensation_angle = -0.264 # [rad]

# SLIDERS   
# motor_velocity_slider = p.addUserDebugParameter("Motor Velocity",-100,100,0)
# episode_done_slider = p.addUserDebugParameter("Episode Done",0,1,0)

# method to map correctly the states of the robot
def robot_output_serial(state, motor_compensation_angle, bar_compensation_angle, motor_angle_range):
    
    out_of_range = False

    # Get the bar angle
    bar_angle = state[0] + bar_compensation_angle
    # Get bar angular velocity
    bar_angular_velocity = state[1]
    # Get the motor angle
    motor_angle = (state[2] + motor_compensation_angle)*180/np.pi

    # Map the motor angle to the correct range
    if motor_angle > motor_angle_range[1] or motor_angle < motor_angle_range[0]:
        out_of_range = True

    # Adjusting the bar angle to map correctly
    bar_angle = bar_angle % (2 * np.pi)  # Normalize the angle to be within 0 to 2π
    if bar_angle > np.pi:
        bar_angle -= 2 * np.pi  # Adjust angles greater than π to be between -π to π

    # round the states to 4 decimal places
    bar_angle = round(bar_angle, 4)
    bar_angular_velocity = round(bar_angular_velocity, 4)
    motor_angle = round(motor_angle, 4)

    return [bar_angle, bar_angular_velocity, motor_angle, out_of_range]

# method to read the fake serial port
def robot_input_serial(p, robotId, motor_joint_index, bar_joint_index, max_motor_speed, motor_angle_range, input_serial):

    # input_serial is [+-percentage of max speed, episode_done]
    # percentage of max speed is [-100,100]
    # episode_done is [0,1]

    speed_percentage = input_serial[0]
    episode_done = input_serial[1]

    # episode_done = p.readUserDebugParameter(episode_done_slider)
    # episode_done = True if episode_done > 0.5 else False

    # Check if the episode is done
    if episode_done:
        # Reset the robot
        reset_random(p, robotId, motor_joint_index, bar_joint_index, motor_angle_range)
    else:
        # Calculate the speed in rad/s
        motor_speed = speed_percentage * max_motor_speed / 100
        # Set the motor speed
        p.setJointMotorControl2(bodyUniqueId=robotId,
                            jointIndex=motor_joint_index,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=motor_speed,
                            )

# method to reset the robot with radom states
def reset_random(p, robotId, motor_joint_index, bar_joint_index, motor_angle_range):

    # Get random states
    bar_angle = np.random.uniform(-np.pi, np.pi)
    bar_angular_velocity = np.random.uniform(-10, 10)
    motor_angle = np.random.uniform(np.deg2rad(motor_angle_range[0]), np.deg2rad(motor_angle_range[1]))

    # Reset the robot to the home position
    p.resetJointState(robotId, motor_joint_index, targetValue=motor_angle)
    p.resetJointState(robotId, bar_joint_index, targetValue=bar_angle)

    # set bar velocity
    p.setJointMotorControl2(bodyUniqueId=robotId,
                            jointIndex=bar_joint_index,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=bar_angular_velocity,
                            force=0
                            )
    
    return [bar_angle, bar_angular_velocity, motor_angle]

reset_random(p, robotId, motor_joint_index, bar_joint_index, motor_angle_range)

# move camera to focus on the robot
p.resetDebugVisualizerCamera(cameraDistance=0.4, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0,0,0.1])

while True:

    # Get the bar angle
    bar_angle = p.getJointState(robotId,bar_joint_index)[0]
    # Get bar angular velocity
    bar_angular_velocity = p.getJointState(robotId,bar_joint_index)[1]
    # Get the motor angle
    motor_angle = p.getJointState(robotId,motor_joint_index)[0]
    
    robot_output = robot_output_serial([bar_angle, bar_angular_velocity, motor_angle], motor_compensation_angle, bar_compensation_angle, motor_angle_range)
    # if out of range, reset the robot
    if robot_output[3]:
        reset_random(p, robotId, motor_joint_index, bar_joint_index, motor_angle_range)
        print("Out of range! Resetting the robot...")

    # read the fake serial port
    random_motor_speed = np.random.uniform(-100, 100)
    robot_input = robot_input_serial(p, robotId, motor_joint_index, bar_joint_index, max_motor_speed, motor_angle_range, [random_motor_speed, 0])
    print(f"Bar angle: {robot_output[0]} [rad], Bar angular velocity: {robot_output[1]} [rad/s], Motor angle: {robot_output[2]} [deg], Out of range: {robot_output[3]}")

    p.stepSimulation()
    # time.sleep(1./240.)

p.disconnect()
