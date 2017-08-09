# Implement the first move model for the Lego robot.
# 02_a_filter_motor
# Claus Brenner, 31 OCT 2012
from math import sin, cos, pi
from pylab import *
from lego_robot import *

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width):
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
        l=motor_ticks[0]*ticks_to_mm
        r=motor_ticks[1]*ticks_to_mm
        alpha=(r-l)/robot_width
        R=l/alpha
        OC=R+robot_width/2
        cx=x-OC*sin(theta)
        cy=y+OC*cos(theta)
        theta1=(theta+alpha)%(math.pi*2)
        x1=cx+OC*sin(theta1)
        y1=cy-OC*cos(theta1)
        return x1,y1,theta1

if __name__ == '__main__':
    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width = 150.0

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Start at origin (0,0), looking along x axis (alpha = 0).
    pose = (0.0, 0.0, 0.0)

    # Loop over all motor tick records generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width)
        filtered.append(pose)

    # Draw result.
    for pose in filtered:
        print(pose)
        # pylab.plot(  x,y, color(blue) shape 'o' )
        plot([p[0] for p in filtered], [p[1] for p in filtered], 'bo')
    show()
