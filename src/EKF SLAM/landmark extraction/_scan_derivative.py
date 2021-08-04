# Compute the derivative of a scan.

from pylab import *
from lego_robot import *

def compute_derivative(scan, min_dist):
    jumps = [ 0 ]
    for i in range(1, len(scan)-1):
        a=scan[i+1]
        b=scan[i-1]
        if a>min_dist and b>min_dist:
            d=(a-b)/2
            jumps.append(d) 
        else:
            jumps.append(0)

    jumps.append(0)
    return jumps


if __name__ == '__main__':

    minimum_valid_distance = 20.0

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_scan.txt")

    # Pick one scan.
    scan_no = int(input("enter the scan number-..."))
    scan = logfile.scan_data[scan_no]

    # Compute derivative, (-1, 0, 1) mask.
    der = compute_derivative(scan, minimum_valid_distance)

    # Plot scan and derivative.
    title("Plot of scan %d" % scan_no)
    plot(scan)
    plot(der)
    show()
