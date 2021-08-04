# Plot the increments of the left and right motor.

from pylab import *
from lego_robot import LegoLogfile

if __name__ == '__main__':

    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    plot(logfile.motor_ticks)
    show()
