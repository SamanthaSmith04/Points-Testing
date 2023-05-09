## Points Test code
## Samantha Smith
## Generates a 2D array of points
## UNITS: meters

#imports
import matplotlib.pyplot as plt
import numpy as np
from rdp import rdp
import time

#main
def main():
    print("=====================================================")
    print("                     Point Tests                     ")
    print("=====================================================")
    #print("LINEAR")
    #linear_test(10, 0.0005, 0.5)

    print("CURVED")
    curve_test2D(-50, 50, 0.0005, 0.01)


#creates a straight, diagonal line 
# y = x
def linear_test(maxYValue, inPointSpacing, outPointSpacing):
    #pointArray = [[0]* 2 for i in range(maxYValue)]
    xpoints = np.arange(0, maxYValue, inPointSpacing)
    ypoints = np.arange(0, maxYValue, inPointSpacing)

    plt.plot(xpoints, ypoints)
    plt.show()

#creates a curved line
#MaxYValue: highest the points can go
#inPointSpacing: spacing between generated points in meters
#outPointSpacing: spacing between the line fit points in meters
"""
    Curve Test Function
    Creates a curved line and then uses the rdp algorithm to fit lines to it
    
    Parameters:
        maxYValue: highest the points can go
        inPointSpacing: spacing between generated points in meters
        outPointSpacing: spacing between the line fit points in meters
    NOTE: For future use, the min and max y values and the point spacing wont be necessary since it will 
    just have to read in the points from a file, rather than generate them
    
"""
def curve_test2D(minYValue, maxYValue, inPointSpacing, outPointSpacing):
    startTime = time.perf_counter()
    #pointArray = [[0]* 2 for i in range(maxYValue)]
    ##initial data points
    xpoints = np.arange(minYValue, maxYValue, inPointSpacing)
    ypoints = np.arange(minYValue, maxYValue, inPointSpacing)
    #xpoints = np.power(xpoints, 3)
    xpoints = np.sin(xpoints)

    ##corrected data points using rdp algorithm
    corrections = rdp(np.column_stack((xpoints, ypoints)), epsilon=outPointSpacing)
    
    print(corrections)  
    endTime = time.perf_counter()
    print("Time to run: " + (endTime - startTime).__str__() + "s")

    ##plot overlaid graphs
    plt.plot(xpoints, ypoints, "ko", markersize=1)
    plt.plot(corrections[:,0], corrections[:,1], "ro", markersize=5)
    plt.plot(corrections[:,0], corrections[:,1], "r", markersize=5)
    plt.text(40000, 00, "Maximum Y values: " + maxYValue.__str__())
    plt.text(40000, 10, "Minimum Y values: " + minYValue.__str__())
    plt.text(40000, 20, "Input point spacing: " + inPointSpacing.__str__())
    plt.text(40000, 30, "Output point spacing: " + outPointSpacing.__str__())
    plt.show()

    ##plot side by side graphs
    plt.subplot(1,2,1)
    plt.plot(xpoints, ypoints, "ko", markersize=1)
    plt.subplot(1,2,2)
    plt.plot(corrections[:,0], corrections[:,1], "ro", markersize=5)
    plt.plot(corrections[:,0], corrections[:,1], "r", markersize=5)
    plt.show()


#main start
if __name__ == '__main__':
    main()