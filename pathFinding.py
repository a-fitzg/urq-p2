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
    @param colour : OPTIONAL - colour of point

    """
    def __init__(self, x, y, *args, **kwargs):
        self.x = x
        self.y = y

        # We might also pass in a colour parameter, this is optional and is not a normal argument
        if len(args) != 0:
            self.colour = args[0]

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_colour(self):
        return self.colour


def get_points_mean(points):
    """
    Get the middle point of a set of points

    @param points: List of points
    @return: Point: Middle point of all points (simple mean)
    """
    sum_x = 0
    sum_y = 0
    i = 0
    for i in range(len(points)):
        sum_x += points[i].get_x()
        sum_y += points[i].get_y()

    ret_point = Point((sum_x / (i + 1)), (sum_y / (i + 1)))

    return ret_point


def parse_points(list_x, list_y, list_colours):
    """
    Translate coordinates list to list of Points

    @param list_x: List of x coordinates
    @param list_y: List of y coordinates
    @param list_colours: List of colours for each point
    @return: List of points
    """
    points = []
    for i in range(len(list_x)):
        points.append(Point(list_x[i], list_y[i], list_colours[i]))

    return points


def cone_pair_midpoint(point1, point2):
    """
    Function to calculate the midpoint between two points.

    @param point1 : First Point
    @param point2 : Second Point

    @return : New Point object, at the midpoint
    """

    xval = (point1.get_x() + point2.get_x()) / 2
    yval = (point1.get_y() + point2.get_y()) / 2
    retpoint = Point(xval, yval)

    return retpoint


def sort_points(listx, listy, colours):
    """
    Given a list of x coords, y coords, and colours, sort the points
    ASSUMING THE TRACK IS CLOCKWISE, AND WE MAKE AN ANTI-CLOCKWISE PATH

    @param listx   : List of points' x coordinates
    @param listy   : List of points' y coordinates
    @param colours : List of points' colours

    @return : (sorted_x, sorted_y, sorted_colours):
                sorted_x : Sorted x coordinates
                sorted_y : Sorted y coordinates
                sorted_colours : Sorted colours
    """

    points = parse_points(listx, listy, colours)
    centre_point = get_points_mean(points)
    start_points = []

    # First, find the 2 start points, they are of colour 0
    for i in points:
        if i.get_colour() == 0:
            start_points.append(i)

    # If we don't have 2 start cones, we can't have a valid start point. Quit
    if len(start_points) != 2:
        return

    sorted_points_pair = {0 : (start_points[0], start_points[1])}

    # Now, we go around anti-clockwise and find a path for one side of the cones,
    # we will look for cones with colour 1


    sorted_listx = []
    sorted_listy = []
    sorted_colours = []


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
    # Dict: {index: (point1, point2)}
    cones = {0: (Point(listx[0], listy[0], 0), Point(listx[1], listy[1], 0))}

    # Iterate through the list of points, assigning all subsequent pairs of points into a tuple
    for i in range(1, (len(listx) // 2)):
        cones[i] = (Point(listx[i * 2], listy[i * 2], colours[i * 2]), \
                    Point(listx[(i * 2) + 1], listy[(i * 2) + 1], colours[(i * 2) + 1]))

    return cones


def get_track_midpoints(points):
    """
    Get a list of midpoints, with their index, in a dictionary
    @param points : Dictionary containing point pairs with indices
    @type points  : Dictionary := {index: (Point, Point)}

    @return       : Dictionary containing midpoints with indices
    """

    # Our dictionary of midpoints, empty for now
    mids = {}

    # Iterate through the cones dict, calculating the midpoint of each one, then putting it in a
    # new dictionary, mapping index to Point (midpoint)
    for i in points:
        mids[i] = cone_pair_midpoint(points.get(i)[0], points.get(i)[1])

    # We might also add a point back at the start line to form a complete circuit
    mids[len(points)] = mids[0]

    return mids


def get_point_list(points):
    """
    Returns a tuple containing a list of x coordinates and a list of y coordinates from a cone
    dictionary

    @param points   : Dictionary mapping point indices to Point objects
    @return         : Tuple containing list of x coordinates, list of y coordinates, and list of colours
    """

    xList = []
    yList = []
    colourList = []

    for i in points:
        xList.append(points.get(i).get_x())
        yList.append(points.get(i).get_y())
        colourList.append(points.get(i).get_colour())

    return xList, yList, colourList


def task1():
    """ Task 1 """

    # Store all the cone pairs as tuples in a dictionary, with its index as a key
    cones = get_point_pairs(CX1, CY1, CC1)

    # Store all the midpoints as dictionary entries, with its index as a key
    midpoints = get_track_midpoints(cones)

    (xList, yList) = get_point_list(midpoints)

    # Plot out existing tracks
    plt.scatter(CX1, CY1, c=CC1)

    # Plot our midpoints
    plt.plot(xList, yList, '-o')
    plt.axis('equal')
    plt.show()


def task2():
    """ Task 2 """
    sorted_points = sort_points(CX2, CY2, CC2)


    #cones = get_point_pairs(CX2, CY2, CC2)

    #(xlist, ylist, colourlist) = get_point_list(cones)

    #plt.scatter(xlist, ylist, c=colourlist)
    plt.scatter(CX2, CY2, c=CC2)
    plt.scatter(centre_point.get_x(), centre_point.get_y())
    plt.axis('equal')
    plt.show()

##------------------------------------------------------------------------------


if __name__ == '__main__':

    #task1()
    task2()


# -------------------------------------------------------------------------------
