## Points Test code
## Samantha Smith
## Generates a 2D or 3D array of points and uses the Ramer-Douglas-Peucker algorithm to fit lines to the points
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
    select_test()
    print("points generated!")



#creates a straight, diagonal line 
# y = x
def linear_test(maxYValue, inPointSpacing, outPointSpacing, fileName):
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
        minYValue: lowest the points can go
        maxYValue: highest the points can go
        inPointSpacing: spacing between generated points in meters
        outPointSpacing: spacing between the line fit points in meters
        useFile: string, if y the output will be saved to a file, if n then the output will be written to the console
    NOTE: For future use, the min and max y values and the point spacing wont be necessary since it will 
    just have to read in the points from a file, rather than generate them
    
"""
def curve_test2D(minYValue, maxYValue, inPointSpacing, outPointSpacing, fileName):
    if (fileName != ""):
        outFile = open(fileName, "w")

    startTime = time.perf_counter()
    #pointArray = [[0]* 2 for i in range(maxYValue)]
    ##initial data points
    xpoints = np.arange(minYValue, maxYValue, inPointSpacing)
    ypoints = np.arange(minYValue, maxYValue, inPointSpacing)

    xpoints = np.sin(xpoints)

    ##corrected data points using rdp algorithm
    corrections = rdp(np.column_stack((xpoints, ypoints)), epsilon=outPointSpacing)
    
    if fileName != "":
        for i in range(len(corrections)):
            outFile.write(corrections[i,0].__str__() + " " + corrections[i,1].__str__() + " " + corrections[i,2].__str__() + "\n")
        outFile.close()
    else:
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
    plt.title("Original Data Points")
    plt.plot(xpoints, ypoints, "ko", markersize=1)
    plt.subplot(1,2,2)
    plt.title("Correction Points")
    plt.plot(corrections[:,0], corrections[:,1], "ro", markersize=5)
    plt.plot(corrections[:,0], corrections[:,1], "r", markersize=5)
    plt.show()

""" 
    Curve Test 3D Function (generated data)
    Creates a curved line in 3D space from generated points and then uses the rdp algorithm to fit lines to it
    Also displays a 2d representation of the points in the xy plane

    Parameters:
        minYValue: lowest the points can go
        maxYValue: highest the points can go
        inPointSpacing: spacing between generated points in meters
        outPointSpacing: spacing between the line fit points in meters
        useFile: string, if y the output will be saved to a file, if n then the output will be written to the console  
"""
def curve_test3D(minYValue, maxYValue, inPointSpacing, outPointSpacing, fileName):
    if (fileName != ""):
        outFile = open(fileName, "w")
    startTime = time.perf_counter()
    fig = plt.figure().add_subplot(projection='3d')
    ##initial data points
    print("Generating points...")
    xpoints = np.arange(minYValue, maxYValue, inPointSpacing)
    ypoints = np.arange(minYValue, maxYValue, inPointSpacing)
    zpoints = np.arange(minYValue, maxYValue, inPointSpacing)

    ###THESE CAN BE EDITED TO GENERATE NEW GRAPHS###
    xpoints = np.sin(xpoints)
    zpoints = np.cos(zpoints)

    ##corrected data points using rdp algorithm
    print("Correcting points...")
    corrections = rdp(np.column_stack((xpoints, ypoints, zpoints)), epsilon=outPointSpacing)
    
    if fileName != "":
        print("Writing to file...")
        for i in range(len(corrections)):
            outFile.write(corrections[i,0].__str__() + " " + corrections[i,1].__str__() + " " + corrections[i,2].__str__() + "\n")
        outFile.close()
    else:
        print(corrections)  
    endTime = time.perf_counter()
    print("Time to run: " + (endTime - startTime).__str__() + "s")
    print("Number of points used in correction: " + len(corrections).__str__())

    ##plot overlaid graphs
    fig.plot3D(xpoints, ypoints, zpoints, "ko", markersize=1)
    fig.plot3D(corrections[:,0], corrections[:,1], corrections[:,2], "ro", markersize=5)
    fig.plot3D(corrections[:,0], corrections[:,1], corrections[:,2], "r", markersize=5)
    
    plt.show()

    ##plot side by side graphs
    plt.subplot(1,3,1)
    plt.title("Original Data Points")
    plt.plot(xpoints, ypoints, "ko", markersize=1)
    plt.subplot(1,3,2)
    plt.title("Correction Points")
    plt.plot(corrections[:,0], corrections[:,1], "ro", markersize=5)
    plt.plot(corrections[:,0], corrections[:,1], "r", markersize=5)
    plt.subplot(1,3,3)
    plt.title("Original and Correction Points Overlaid")
    plt.plot(xpoints, ypoints, "ko", markersize=1)
    plt.plot(corrections[:,0], corrections[:,1], "ro", markersize=5)
    plt.plot(corrections[:,0], corrections[:,1], "r", markersize=5)
    plt.tight_layout(pad=3)
    plt.show()
    

"""
    Curve Test 3D Function (from a file)
    Reads in a set of points from a file selected by the user and then uses the rdp algorithm to fit lines to it
    Is able to output data to either a file selected by the user or the console

    Parameters:
        outPointSpacing: spacing between the line fit points in meters
        inputFileName: name of the file to read in data from
        outputFileName: name of the file to output data to
"""
def curve_test3DfromFile(outPointSpacing, inputFileName, outputFileName):
    xpoints = []
    ypoints = []
    zpoints = []
    #opens the output file if the user wants to save the data
    if (outputFileName != ""):
        outFile = open(outputFileName, "w")

    startTime = time.perf_counter()
    fig = plt.figure().add_subplot(projection='3d')

    #read in data from the input file
    print("Reading in data from " + inputFileName + "...")
    inFile = open(inputFileName, "r")
    for line in inFile:
        line = line.split()
        xpoints.append(float(line[0]))
        ypoints.append(float(line[1]))
        zpoints.append(float(line[2]))
    inFile.close()

    print("Computing corrected points...")
    ##corrected data points using rdp algorithm
    corrections = rdp(np.column_stack((xpoints, ypoints, zpoints)), epsilon=outPointSpacing)
    
    #write corrected points to the output file
    if outputFileName != "":
        print("Writing corrected points to " + outputFileName + "...")
        for i in range(len(corrections)):
            outFile.write(corrections[i,0].__str__() + " " + corrections[i,1].__str__() + " " + corrections[i,2].__str__() + "\n")
        outFile.close()
    else:
        print(corrections)  

    endTime = time.perf_counter()
    print("Time to run: " + (endTime - startTime).__str__() + "s")
    print("Number of points used in correction: " + len(corrections).__str__())

    ##plot overlaid graphs
    fig.plot3D(xpoints, ypoints, zpoints, "ko", markersize=1)
    fig.plot3D(corrections[:,0], corrections[:,1], corrections[:,2], "ro", markersize=5)
    fig.plot3D(corrections[:,0], corrections[:,1], corrections[:,2], "r", markersize=5)
    
    plt.show()

    ##plot side by side graphs
    plt.subplot(1,3,1)
    plt.title("Original Data Points")
    plt.plot(xpoints, ypoints, "ko", markersize=1)
    plt.subplot(1,3,2)
    plt.title("Correction Points")
    plt.plot(corrections[:,0], corrections[:,1], "ro", markersize=5)
    plt.plot(corrections[:,0], corrections[:,1], "r", markersize=5)
    plt.subplot(1,3,3)
    plt.title("Original and Correction Points Overlaid")
    plt.plot(xpoints, ypoints, "ko", markersize=1)
    plt.plot(corrections[:,0], corrections[:,1], "ro", markersize=5)
    plt.plot(corrections[:,0], corrections[:,1], "r", markersize=5)
    plt.tight_layout(pad=3)
    plt.show()


"""
    Select Test Function
    Allows the user to select which test they want to run and 
    which values should be used as input
"""
def select_test():
    inFileName = ""
    outFileName = ""
    while True:
        print("Select a test to run:")
        print("1. Linear Test")
        print("2. 2D Curve Test")
        print("3. 3D Curve Test")
        print("4. Exit")
        selection = input("Enter selection: ")
        if (selection == "4"):
            exit()
        if (int(selection) > 4 or int(selection) < 1):
            print("Invalid selection, please try again")
            continue
        print("=====================================")
        print("Write output to file? (y/n)")
        fileChoice = input()
        if (fileChoice == "y"):
            print("Enter file name: ")
            outFileName = input()
        print("=====================================")
        print("Use an input file? (y/n)")
        fileChoice = input()
        if (fileChoice == "y"):
            print("Enter file name: ")
            inFileName = input()
        else:
            print("=====================================")
            print("Values for data generation (units - meters): ")
            print("Minimum Y Value for data generation: ")
            miny = float(input())
            print("Maximum Y Value for data generation: ")
            maxy = float(input())
            print("Input point spacing: ")
            inPointSpacing = float(input())
        print("Output point spacing: ")
        outPointSpacing = float(input())
        if selection == "1":
            print("generating points...")   
            linear_test(maxy, inPointSpacing, outPointSpacing, outFileName)
            break
        elif selection == "2":
            print("generating points...")
            curve_test2D(miny, maxy, inPointSpacing, outPointSpacing, outFileName)
            break
        elif selection == "3":
            if (inFileName != ""):
                curve_test3DfromFile(outPointSpacing, inFileName, outFileName)
            else:
                curve_test3D(miny, maxy, inPointSpacing, outPointSpacing, outFileName)
            break
        
    print("Done!")
    print("=====================================")
    if inFileName == "":
        print("Minimim Y Value for data generation: " + miny.__str__())
        print("Maximum Y Value for data generation: " + maxy.__str__())
        print("Input point spacing: " + inPointSpacing.__str__())
    print("Output point spacing: " + outPointSpacing.__str__())
    print("=====================================")

    print("Generate a new graph? (y/n)")
    choice = input()
    if choice == "y":
        select_test()
        plt.close('all')
    else:
        exit()
    

#main start
if __name__ == '__main__':
    main()