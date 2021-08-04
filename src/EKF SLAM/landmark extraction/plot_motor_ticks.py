# Plot the ticks from the left and right motor.

from pylab import *

if __name__ == '__main__':
    
    f = open("robot4_motors.txt")
    left_list = []
    right_list = []
    for l in f:
        sp = l.split()
        left_list.append(int(sp[2]))
        right_list.append(int(sp[6]))
        
    plot(left_list)
    plot(right_list)
    show()
