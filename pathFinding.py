##   UQR DV New Member pathFinding task
##   code written by: Mitchell Anderson
##                    mitchell.anderson@uqracing.com
##   code mangled by: Alexander FitzGerald
##                    xanderfitzg2000@gmail.com


##------------------------------Import Statements-------------------------------
import numpy as np
import math
import matplotlib.pyplot as plt

##------------------------------------------------------------------------------

##------------------------------TASK 1 CONE LAYOUT------------------------------
CX1 = [1, 1, 2, 2, 3, 3, 4, 4.7, 4, 5, 4, 4.7, 3, 3, 2, 2, 1, 0.3, 1, 0]
CY1 = [1, 0, 1, 0, 1, 0, 1, 0.3, 1.1, 1.1, 1.2, 1.9, 1.2, 2.2, 1.2, 2.2, 1.2, 1.9, 1.1, 1.1]
CC1 = [0, 0, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2]
##------------------------------------------------------------------------------

##------------------------------TASK 2 CONE LAYOUT------------------------------
CX2 = [2, 2, 3, 3, 4, 4, 0, 4.7, 5, 2, 4.7, 1, 0.3, 1, 1, 4, 3, 3, 2, 1]
CY2 = [1.2, 1, 0, 1, 1.2, 1, 1.1, 1.9, 1.1, 0, 0.3, 0, 1.9, 1.1, 1, 1.1, 1.2, 2.2, 2.2, 1.2]
CC2 = [1, 1, 2, 1, 1, 1, 2, 2, 2, 2, 2, 0, 2, 1, 0, 1, 1, 2, 2, 1]


##------------------------------- My code  ------------------------------

class Point:
    """
    Class representing a point on the track

    @param x : x coordinate of point
    @param y : y coordinate of point

    """
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y


def cone_pair_midpoint(point1, point2):
    """
    Function to calculate the midpoint between two points.

    @param point1 : First Point
    @param point2 : Second Point

    @return : New Point object, at the midpoint
    """

    xval = (point1.get_x + point2.get_x) / 2
    yval = (point1.get_y + point2.get_y) / 2
    retpoint = Point(xval, yval)

    return retpoint


def get_point_pairs(listx, listy, colours):
    """
    Populates a dictionary with Point pairs, with an order
    REQUIRES THAT THE LIST BE ORDERED

    @param listx   : List of x coordinates
    @param listy   : List of y coordinates
    @param colours : List of cone colours

    @return : Dictionary with indices as keys, and Point tuples as entries
    """


    # In the ordered list, the first point is the start cones
    cones = {0: (Point(listx[0], listy[0]), Point(listx[1], listy[1]))}
    print(cones)

    dictIndex = 0
    # Iterate through the list of ponits
    for i in range(1, len(listx)):
        cones[i] = {i: (Point(listx[i], listy[i]), Point(listx[i + 1], listy[i + 1]))}
        pass

    return cones

def task1():
    """ Task 1 """

    # Store all the cone pairs as tuples in a dictionary, with its index as a key
    cones = get_point_pairs(CX1, CY1, CC1)

    # Store all the midpoints as dictionary entries, with its index as a key
    midpoints = {}

    # Go through all the point pairs in the track
    for i in range(len(CX1)):

        pass

##------------------------------------------------------------------------------


if __name__ == '__main__':

    task1()

    plt.scatter(CX1, CY1, c=CC1)
    plt.axis('equal')
    plt.show()

# -------------------------------------------------------------------------------
