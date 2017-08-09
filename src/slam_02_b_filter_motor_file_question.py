# Implement the second move model for the Lego robot.
# The difference to the first implementation is:
# - added a scanner displacement
# - added a different start pose (measured in the real world)
# - result is now output to a file, as "F" ("filtered") records.
#
# 02_b_filter_motor_file
# Claus Brenner, 09 NOV 2012
from math import sin, cos, pi
from lego_robot import *

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width,
                scanner_displacement):
    x,y,theta=old_pose
    # Find out if there is a turn at all.
    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.
        l=ticks_to_mm*motor_ticks[0]
        x1=x+l*cos(theta)
        y1=y+l*sin(theta)
        theta1=theta
        return x1,y1,theta1
    else:
        # Turn. Compute alpha, R, etc.
        
        # First modify the the old pose to get the center (because the
        #   old pose is the LiDAR's pose, not the robot's center pose).
        # Second, execute your old code, which implements the motion model
        #   for the center of the robot.
        # Third, modify the result to get back the LiDAR pose from
        #   your computed center. This is the value you have to return.
        x=x-scanner_displacement*sin(theta)
        y=y-scanner_displacement*cos(theta)

        l=motor_ticks[0]*ticks_to_mm
        r=motor_ticks[1]*ticks_to_mm
        alpha=(r-l)/robot_width
        R=l/alpha
        OC=R+robot_width/2
        cx=x-OC*sin(theta)
        cy=y+OC*cos(theta)
        theta1=(theta+alpha)%(pi*2)
        x1=cx+OC*sin(theta1)
        y1=cy-OC*cos(theta1)

        x1=x1+scanner_displacement*sin(theta1)
        y1=y1+scanner_displacement*cos(theta1)
        return x1,y1,theta1



if __name__ == '__main__':
    # Empirically derived distance between scanner and assumed
    # center of robot.
    scanner_displacement = 30.0

    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width = 173.0

    # Measured start position.
    pose = (1850.0, 1897.0, 213.0 / 180.0 * pi)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width,
                           scanner_displacement)
        filtered.append(pose)

    # Write all filtered positions to file.
    f = open("poses_from_ticks.txt", "w")
    for pose in filtered:
        f.write("F %f %f %f\n" % pose)
        #print >> f, 
    f.close()
