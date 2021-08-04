
from math import sin, cos, pi
from lego_robot import *

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width,
                scanner_displacement):
    l1=motor_ticks[0]*ticks_to_mm #right
    l2=motor_ticks[1]*ticks_to_mm #left
    x=old_pose[0]
    y=old_pose[1]
    theta=old_pose[2]
    # Find out if there is a turn at all.
    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.
        x=x+l1*cos(theta)
        y=y+l1*sin(theta)

        # --->>> Use your previous implementation.
        # Think about if you need to modify your old code due to the
        # scanner displacement?
        #It doesn't matter because 30 gets subtracted on both side and then gets added on both side.
        
        return (x, y, theta)

    else:
        # Turn. Compute alpha, R, etc.
        alpha=(l1-l2)/robot_width
        R=l2/alpha

        # --->>> Modify your previous implementation.
        # First modify the the old pose to get the center (because the
        #   old pose is the LiDAR's pose, not the robot's center pose).
        x=x-scanner_displacement*cos(theta)
        y=y-scanner_displacement*sin(theta)
        # Second, execute your old code, which implements the motion model
        #   for the center of the robot.
        x=x+(R+robot_width/2)*(-sin(theta)+sin(theta+alpha))
        y=y+(R+robot_width/2)*(cos(theta)-cos(theta+alpha))
        theta=(theta+alpha)%(2*pi)
        # Third, modify the result to get back the LiDAR pose from
        x=x+scanner_displacement*cos(theta)
        y=y+scanner_displacement*sin(theta)
        #   your computed center. This is the value you have to return.
        return (x, y, theta)

if __name__ == '__main__':
    # Empirically derived distance between scanner and assumed
    # center of robot.
    scanner_displacement = 30.0

    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width = 150.0

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
    f = open("poses_from_ticks.txt", "w+")
    for pose in filtered:
        f.write("F")
        f.writelines(str(pose))
        f.write("\n")
    
    f.close()
