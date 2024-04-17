import math
import numpy as np

from controller import Robot # type: ignore

from autopilot import flightController

if __name__ == '__main__':

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

    # Init states
    prev_rM = np.array([0,0,0])
    prev_time = 0.0
    initFlag = True
    g = np.array([0,0,-9.81])

    # Init controller
    FC = flightController()

    # Waypoints
    xWay = np.array([-1]) #np.array([-1,-3,-4,8])
    yWay = np.array([2]) #np.array([2,7,-3,5])
    i = 0

    while robot.step(timestep) != -1:
        
        currTime = robot.getTime()
        dt = currTime - prev_time
        print(f"time = {robot.getTime()}")
        

        if initFlag:
            prev_rM = np.array([gps.getValues()[0], gps.getValues()[1], gps.getValues()[2]])
            prev_time = robot.getTime()
            initFlag = False

        # Sensor data
        curr_att = np.array([imu.getRollPitchYaw()[0], imu.getRollPitchYaw()[1], imu.getRollPitchYaw()[2]]) # [roll, pitch, yaw]
        curr_attRate = np.array([gyro.getValues()[0], gyro.getValues()[1], gyro.getValues()[2]]) # [rollRate, pitchRate, yawRate]
        rM = np.array([gps.getValues()[0], gps.getValues()[1], gps.getValues()[2]]) # [x, y, z]
        vM = (rM[0] - prev_rM[0])/dt, (rM[1] - prev_rM[1])/dt, (rM[2] - prev_rM[2])/dt # [vx, vy, vz]

        # Camera data
        cameraData = camera.getImage()
        gray = camera.imageGetGray(cameraData, camera.getWidth(), 5, 10)
        red = camera.imageGetRed(cameraData, camera.getWidth(), 5, 10)
        green = camera.imageGetGreen(cameraData, camera.getWidth(), 5, 10)
        blue = camera.imageGetBlue(cameraData, camera.getWidth(), 5, 10)

        # Desired states
        roll_des = 0.0
        pitch_des = 0.0
        yaw_des = 0.0
        rollRate_des = 0.0
        pitchRate_des = 0.0
        yawRate_des = 0.0
        x_des = 0.0
        y_des = 0.0
        h_des = 1
    
        ############### TEST CODE ###############
        if currTime > 5:
            x_des = xWay[i]
            y_des = yWay[i]
            print(f"Next Waypoint = {[x_des,y_des]}")
            if np.linalg.norm(rM[0:2] - [x_des,y_des]) < 0.05 and i < len(xWay) - 1:
                i += 1
        #########################################
 
        print(f"roll, pitch, yaw = {curr_att}")
        # print(f"roll_rate, pitch_rate, yaw_rate = {curr_attRate}")
        print(f"X, Y, Z = {rM}")
        # print(f"att_des = {att_des}")
        attRate_des = curr_attRate
        currStates = np.array([curr_att[0],curr_attRate[0],curr_att[1],curr_attRate[1],curr_att[2],curr_attRate[2],
                               rM[2],vM[2],rM[0],vM[0],rM[1],vM[1]])
        desiredStates = np.array([roll_des,rollRate_des,pitch_des,pitchRate_des,yaw_des,yawRate_des,
                               h_des,0,x_des,0,y_des,0])
        
        motor_power = FC.backstep(desiredStates, currStates)

        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        prev_time = robot.getTime()
        prev_rM = rM
        