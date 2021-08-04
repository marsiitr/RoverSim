# Plot a scan of the robot using matplotlib.
#TOTAL SCAN = 660

from pylab import *
from lego_robot import *
n=int(input("enter the scan number-.."))
# Read the logfile which contains all scans.
logfile = LegoLogfile()
logfile.read("robot4_scan.txt")

# Plot one scan.
plot(logfile.scan_data[n])
show()
