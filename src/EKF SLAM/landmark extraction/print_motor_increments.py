# Print the increments of the left and right motor.
# Now using the LegoLogfile class.

from lego_robot import LegoLogfile

if __name__ == '__main__':

    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    for i in range(20):
        print(logfile.motor_ticks[i])
