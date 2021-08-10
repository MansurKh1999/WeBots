"""driveMyMiniRobot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import sys
import math



if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    timestep = 64
    max_speed = 6.25
    sideNum = 4
    sideLen = 0.25
    wheel_radius = 0.025
    linear_speed = wheel_radius * max_speed
    straightTime = sideLen/linear_speed
    rotationTime = math.pi*0.045/(2*6.25*0.025)
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)
    
    left_motor = robot.getDevice('motor1')
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor = robot.getDevice('motor2')
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    startTime = robot.getTime()
    rotationStartTime = startTime + straightTime # 1.6
    rotationEndTime = rotationStartTime + rotationTime #1.85
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        
        currentTime = robot.getTime()
        left_speed = max_speed
        right_speed = max_speed       
           
       
        
        if  rotationStartTime < currentTime < rotationEndTime:
            left_speed = -max_speed
            right_speed = max_speed
            
            
        elif currentTime >rotationEndTime:
            rotationStartTime = currentTime + straightTime
            rotationEndTime = rotationStartTime + rotationTime
            
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        # if currentTime > startTime + straightTime:
            # left_motor.setVelocity(0)
            # right_motor.setVelocity(0)
                    
        # if :
            # left_motor.setVelocity(0)
            # right_motor.setVelocity(right_speed)
        # else:
            # left_motor.setVelocity(left_speed)
            # right_motor.setVelocity(right_speed)
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
    
        # Process sensor data here.
    
        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        pass
    
    # Enter here exit cleanup code.
