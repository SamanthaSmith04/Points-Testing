#! /usr/bin/env python3

import sys
import os
#sources ros so that packages can be found
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

## Points Test code
## Samantha Smith
## Generates a 2D or 3D array of points and uses the Ramer-Douglas-Peucker algorithm to fit lines to the points
## UNITS: meters

#imports
import matplotlib.pyplot as plt
import numpy as np
import rdp_algorithm
import time
import rospy

global file_path
file_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))) + "/points_data/"

#main
def main():
    print("=====================================================")
    print("                     Point Tests                     ")
    print("=====================================================")
    select_test()
    
    print("points generated!")


"""
    Curve Test 3D
    Reads/Generates a list of points to process with the rdp algorithm and plots them using matpltlib
    Outputs to either a file or the console
    Parameters:
        outPointSpacing: the maximum spacing between the original data points and the line between the correction points
        inputFileName: the name of the file to read the points from
        outputFileName: the name of the file to write the corrected points to
        minYValue: the minimum y value to generate points for
        maxYValue: the maximum y value to generate points for
        inPointSpacing: the spacing between the points to generate
"""
def curve_test3D(outPointSpacing, inputFileName, outputFileName, minYValue, maxYValue, inPointSpacing):
    points =[]
    startTime = time.perf_counter()
    fig = plt.figure().add_subplot(projection='3d')

    if (inputFileName != ""):
        points = get_points_from_file(inputFileName)
    else:
        print("Generating points...")
        points = generatePoints(minYValue, maxYValue, inPointSpacing)
        #xpoints, ypoints, zpoints = generate_raster(inPointSpacing)
        print("Points generated!")

    corrections = correctPoints(points, outPointSpacing)

    print("Calculating delta values...")
    max = delta(points, corrections)

    
    #write corrected points to the output file
    if outputFileName != "":
        deltaOutput = outputFileName.split(".")[0] + "_delta.txt"
        write_corrections_to_file(corrections, outputFileName)
        write_delta_to_file(deltaOutput, max)
    else:
        print(corrections)
        for i in range(len(corrections)-1):
            print(corrections[i,0].__str__() + " " + corrections[i,1].__str__() + " " + corrections[i,2].__str__())
            print("delta: " + max[i].__str__())
        print(corrections[len(corrections)-1,0].__str__() + " " + corrections[len(corrections)-1,1].__str__() + " " + corrections[len(corrections)-1,2].__str__())

    endTime = time.perf_counter()
    print("Time to run: " + (endTime - startTime).__str__() + "s")
    print("Number of points used in correction: " + len(corrections).__str__())

    ##plot overlaid graphs
    fig.plot3D(points[:,0], points[:,1], points[:,2], "ko", markersize=1)
    fig.plot3D(corrections[:,0], corrections[:,1], corrections[:,2], "ro", markersize=5)
    fig.plot3D(corrections[:,0], corrections[:,1], corrections[:,2], "r", markersize=5)
    
    plt.show()

    ##plot side by side graphs
    plt.subplot(1,3,1)
    plt.title("Original Data Points")
    plt.plot(points[:,0], points[:,1], "ko", markersize=1)
    plt.subplot(1,3,2)
    plt.title("Correction Points")
    plt.plot(corrections[:,0], corrections[:,1], "ro", markersize=5)
    plt.plot(corrections[:,0], corrections[:,1], "r", markersize=5)
    plt.subplot(1,3,3)
    plt.title("Original and Correction Points Overlaid")
    plt.plot(points[:,0], points[:,1], "ko", markersize=1)
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
    miny = 0
    maxy = 0
    inPointSpacing = 0

    inFileName = rospy.get_param('/points_corrector/inputFile')

    outFileName = rospy.get_param('/points_corrector/outputFile')

    outPointSpacing = rospy.get_param('/points_corrector/outPointSpacing')
    if (inFileName == ""):
        print("Values for data generation (units - meters): ")
        print("Minimum Y Value for data generation: ")
        miny = float(input())
        print("Maximum Y Value for data generation: ")
        maxy = float(input())
        print("Input point spacing: ")
        inPointSpacing = float(input())
    if (outPointSpacing == ""):
        print("Output point spacing: ")
        outPointSpacing = float(input())

    curve_test3D(outPointSpacing, inFileName, outFileName, miny, maxy, inPointSpacing)
    
"""
    Delta function
    Calculates the delta values for each correction point
    Parameters:
        points: the original data points
        corrections: the corrected data points
"""
def delta(points, corrections):
    index = 0
    max = np.zeros(len(corrections) - 1)
    for cPos in range(len(corrections) - 1):
        while points[index,0] != corrections[cPos+1,0] or points[index,1] != corrections[cPos+1,1] or points[index,2] != corrections[cPos+1,2]:
            dist = rdp_algorithm.perpendicular_distance(points[index], corrections[cPos], corrections[cPos+1])
            if (abs(dist) > abs(max[cPos])):
                max[cPos] = dist
            index += 1
    print("Delta values calculated!")
    return max

"""
    Generate Points
    Generates a set of points to be used in the test
    Parameters:
        minYValue: the minimum value for the y axis
        maxYValue: the maximum value for the y axis
        inPointSpacing: the spacing between points
"""
def generatePoints(minYValue, maxYValue, inPointSpacing):
    xpoints = np.arange(minYValue, maxYValue, inPointSpacing)
    ypoints = np.arange(minYValue, maxYValue, inPointSpacing)
    zpoints = np.arange(minYValue, maxYValue, inPointSpacing)

    ### THESE CAN BE EDITED TO GENERATE NEW GRAPHS ###
    ypoints = ypoints**1.001
    xpoints = np.sin(xpoints)
    
    return np.column_stack((xpoints, ypoints, zpoints))

"""
    Correct Points
    Corrects the points using the rdp algorithm
    Parameters:
        points: the points to be corrected
        outPointSpacing: the maximum distance between the original points and the correction line
"""
def correctPoints(points, outPointSpacing):
    print("Computing corrected points...")
    ##corrected data points using rdp algorithm
    corrections = rdp_algorithm.rdp_run(points, epsilon=outPointSpacing)
    print("Correction points generated!")
    return corrections

"""
    Generate Raster
    Generates a raster of points to be used in the test
    Parameters:
        inPointSpacing: the spacing between points
"""
def generate_raster(inPointSpacing):
    segmentx1 = np.arange(0, 3, inPointSpacing)
    seg1len = len(segmentx1)
    zeros = np.zeros(seg1len)
    segmenty2 = np.arange(0, 3, inPointSpacing)
    segmentx2 = np.empty(len(segmenty2))
    segmentx2 = np.full(len(segmenty2), segmentx1[-1])

    segmentx3 = np.arange(segmentx2[-1], 5, inPointSpacing)
    segmenty3 = np.empty(len(segmentx3))
    segmenty3 = np.full(len(segmentx3), segmenty2[-1])


    xpoints = np.concatenate((segmentx1, segmentx2, segmentx3))
    ypoints = np.concatenate((zeros, segmenty2, segmenty3))
    zpoints = np.zeros(len(xpoints))
    return np.column_stack((xpoints, ypoints, zpoints))

"""
    Read points from a file
    Parameters:
        inputFileName: the name of the file to read from
"""
def get_points_from_file(inputFileName):
    xpoints = []
    ypoints = []
    zpoints = []
    #read in data from the input file
    print("Reading in data from " + inputFileName + "...")
    inFile = open(file_path + inputFileName, "r")
    for line in inFile:
        line = line.split()
        xpoints.append(float(line[0]))
        ypoints.append(float(line[1]))
        zpoints.append(float(line[2]))
    inFile.close()
    return np.column_stack((xpoints, ypoints, zpoints))

"""
    Write corrections to a file
    Outputs the corrected points to a file
    Parameters:
        corrections: the corrected points
        outputFileName: the name of the file to write to
"""
def write_corrections_to_file(corrections, outputFileName):
    outFile = open(file_path + outputFileName, "w")
    print("Writing corrected points to " + outputFileName + "...")
    for i in range(len(corrections)):
        outFile.write(corrections[i,0].__str__() + " " + corrections[i,1].__str__() + " " + corrections[i,2].__str__() + "\n")
    outFile.close()

"""
    Write delta values to a file
    Outputs the delta values to a file
    Parameters:
        deltaOutput: the name of the file to write to
        deltas: the delta values
"""
def write_delta_to_file(deltaOutput, deltas):
    deltaFile = open(file_path + deltaOutput, "w")
    for i in range (len(deltas)):
        deltaFile.write(deltas[i].__str__() + "\n")
    deltaFile.close()


#main start
if __name__ == '__main__':
    main()