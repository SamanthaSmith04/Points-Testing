## Points Test code
## Samantha Smith
## Generates a 2D array of points
## UNITS: meters

#imports
import matplotlib.pyplot as plt
import numpy as np
from rdp import rdp

#main
def main():
    print("=====================================================")
    print("                     Point Tests                     ")
    print("=====================================================")
    #print("LINEAR")
    #linear_test(10, 0.0005, 0.5)

    print("CURVED")
    curve_test(5000, 0.0005, 0.1)


#creates a straight, diagonal line 
# y = x
def linear_test(numberOfPoints, inPointSpacing, outPointSpacing):
    #pointArray = [[0]* 2 for i in range(numberOfPoints)]
    xpoints = np.arange(0, numberOfPoints*inPointSpacing, inPointSpacing)
    ypoints = np.arange(0, numberOfPoints*inPointSpacing, inPointSpacing)

    plt.plot(xpoints, ypoints)
    plt.show()

#creates a curved line
#y = sqrt(x)
def curve_test(numberOfPoints, inPointSpacing, outPointSpacing):
    #pointArray = [[0]* 2 for i in range(numberOfPoints)]
    xpoints = np.arange(0, numberOfPoints*inPointSpacing, inPointSpacing)
    
    ypoints = np.arange(0, numberOfPoints*inPointSpacing, inPointSpacing)
   
    xpoints = np.power(xpoints, 3)
    plt.subplot(1,2,1)
    plt.plot(xpoints, ypoints)

    corrections = rdp(np.column_stack((xpoints, ypoints)), epsilon=outPointSpacing)
    print(corrections)  
    plt.subplot(1,2,2)
    plt.plot(corrections[:,0], corrections[:,1])
    plt.show()
    


#main start
if __name__ == '__main__':
    main()