from math import sin, cos, pi
from pylab import *
from lego_robot import *

def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width):
    l1=motor_ticks[0]*ticks_to_mm 
    l2=motor_ticks[1]*ticks_to_mm 
    x=old_pose[0]
    y=old_pose[1]
    theta=old_pose[2]
    
    if motor_ticks[0] == motor_ticks[1]:
        x=x+l1*cos(theta)
        y=y+l1*sin(theta)
        return (x, y, theta)

    else:
        alpha=(l1-l2)/robot_width
        R=l2/alpha
        x=x+(R+robot_width/2)*(-sin(theta)+sin(theta+alpha))
        y=y+(R+robot_width/2)*(cos(theta)-cos(theta+alpha))
        theta=(theta+alpha)%(2*pi)
        return (x, y, theta)

if __name__ == '__main__':
    ticks_to_mm = 0.349
    robot_width = 150.0
    in_x=30
    in_y=0

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

   
    pose = (0.0,0.0,0.0)
    
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width)
        filtered.append(pose)

    
    for pose in filtered:
        print(pose)
        plot([p[0] for p in filtered], [p[1] for p in filtered], 'bo')
    show()
