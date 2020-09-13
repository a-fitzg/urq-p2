##   UQR DV New Member pathFinding task
##   code written by: Mitchell Anderson
##                    mitchell.anderson@uqracing.com
##   code mangled by: Alexander FitzGerald
##                    xanderfitzg2000@gmail.com


##------------------------------Import Statements-------------------------------
import numpy as np
import math
import cmath
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

# Constants
TURN_START = 0
TURN_STOP = 1


class Event:
    """
    Class for representing an event that happens on the track

    @param type : Event Type
    @param time : Event time (along linspace)
    """
    def __init__(self, type, time=None, location=None):

        self.type = type

        if time is not None:
            self.time = time

        if location is not None:
            self.location = location


class RacingCurve:
    """
    Class representing a curve on a track (typically taken by a vehicle)

    @param start : Point: start point

    @param inner_bounds : List of Points - Track's inner bounds (OPTIONAL)
    @param outer_bounds : List of Points - Track's outer bounds (OPTIONAL)
    @param ls : Linspace - Linear space for indexing/parametrising curve (OPTIONAL)
    @param end_time : End time (in linspace) of the track (OPTIONAL)
    @param events : List of Event objects (OPTIONAL)
    """
    def __init__(self, start, inner_bounds=None, outer_bounds=None, ls=None, end_time=None, events=None):
        self.start = start

        self.direction = 0
        self.speed = 0
        self.end = 0

        # {t: (Point, direction, speed)}
        self.path = {}

        # OPTIONAL VALUES
        if inner_bounds is not None:
            self.inner_bounds = inner_bounds
        else:
            self.inner_bounds = 0

        if outer_bounds is not None:
            self.outer_bounds = outer_bounds
        else:
            self.outer_bounds = 0

        if ls is not None:
            self.ls = ls
        else:
            self.ls = np.linspace(0, )

        if end_time is not None:
            self.end_time = end_time
        else:
            self.step_size = 0

        if events is not None:
            self.events = events
        else:
            self.events = 0


class Point:
    """
    Class representing a point on the track

    @param x      : x coordinate of point
    @param y      : y coordinate of point
    @param colour : OPTIONAL - colour of point

    """
    def __init__(self, x, y, colour=None):
        self.x = x
        self.y = y
        self.colour = colour

        # We might also pass in a colour parameter. This is optional, and we need to check if it has been given
        if colour is not None:
            self.colour = colour

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_colour(self):
        return self.colour

    def compare_to(self, other):
        """
        Compares this point to another point, return true if it is at the same location, false otherwise

        @param other : Other Point
        @return : True if same location as other point, false otherwise
        """
        return True if ((other.get_x() == self.get_x()) and (other.get_y() == self.get_y())) else False


def get_points_mean(points):
    """
    Get the middle point of a set of points

    @param points : List of points

    @return: Point: Middle point of all points (simple mean)
    """
    sum_x = 0
    sum_y = 0
    i = 0
    for i in range(len(points)):
        sum_x += points[i].get_x()
        sum_y += points[i].get_y()

    # Simple arithmetic mean of x and y coordinates
    ret_point = Point((sum_x / (i + 1)), (sum_y / (i + 1)))

    return ret_point


def parse_points(list_x, list_y, list_colours):
    """
    Translate coordinates list to list of Points

    @param list_x       : List of x coordinates
    @param list_y       : List of y coordinates
    @param list_colours : List of colours for each point

    @return: List of points
    """
    points = []
    for i in range(len(list_x)):
        points.append(Point(list_x[i], list_y[i], list_colours[i]))

    return points


def find_distance(origin, dest):
    """
    Find the (Euclidean) distance between 2 Points

    @param origin : Origin point (reference)
    @param dest   : Point to find distance to

    @return : Euclidean distance to point
    """
    x = abs(origin.get_x() - dest.get_x())
    y = abs(origin.get_y() - dest.get_y())
    return math.sqrt(math.pow(x, 2) + math.pow(y, 2))


def find_closest(origin, points):
    """
    Find the Point that is closest to a given Point

    @param origin : Origin (reference) point
    @param points : List of Points

    @return : Point: whichever point is closest
    """
    closest = 0
    closest_distance = -1
    for i in points:
        # First run-through of the loop, closest is this current point
        if closest_distance == -1:
            closest = i
            closest_distance = find_distance(origin, i)
        # Check if we found a closer point
        elif (find_distance(origin, i)) < closest_distance:
            closest = i

    return closest


def find_angle(origin, dest):
    """
    Find the angle from an origin to a destination point (in degrees). Results from 0 - 360

    @param origin : Origin point (reference)
    @param dest   : Destination point to measure to

    @return : Angle to destination point (from +x axis)
    """
    x = dest.get_x() - origin.get_x()
    y = dest.get_y() - origin.get_y()

    result = math.atan2(y, x)
    if result < 0:
        result += 2 * math.pi

    return math.degrees(result)


def cone_pair_midpoint(point1, point2):
    """
    Function to calculate the midpoint between two points.

    @param point1 : First Point
    @param point2 : Second Point

    @return       : New Point object, at the midpoint
    """

    xval = (point1.get_x() + point2.get_x()) / 2
    yval = (point1.get_y() + point2.get_y()) / 2
    retpoint = Point(xval, yval)

    return retpoint


def sort_points(listx, listy, colours):
    """
    Given a list of x coords, y coords, and colours, sort the points
    ASSUMING THE TRACK IS CIRCULAR/OVAL, AND WE MAKE AN ANTI-CLOCKWISE PATH

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
    blue_cones = []
    yellow_cones = []

    # First, find the 2 start points, they are of colour 0
    for i in points:
        if i.get_colour() == 0:
            start_points.append(i)

    # If we don't have 2 start cones, we can't have a valid start point. Quit
    if len(start_points) != 2:
        return

    # We need to find the closest starting cone to the midpoint (this will be part of the blue list)
    closest_start_cone = find_closest(centre_point, [start_points[0], start_points[1]])
    other_start_cone = start_points[0] if closest_start_cone.compare_to(start_points[1]) else start_points[1]

    start_angle_blue = find_angle(centre_point, closest_start_cone)
    start_angle_yellow = find_angle(centre_point, other_start_cone)

    blue_cones.append(closest_start_cone)
    yellow_cones.append(other_start_cone)

    # We separate the cones into inside and outside cone groups
    for i in points:
        if i.get_colour() == 1:
            blue_cones.append(i)
        elif i.get_colour() == 2:
            yellow_cones.append(i)

    sorted_points_pair = {0: (start_points[0], start_points[1])}

    # We find the angle to each of the blue cones from the centre_point
    cone_bearings_blue = {}
    cone_bearings_yellow = {}
    for i in blue_cones:
        cone_bearings_blue[i] = find_angle(centre_point, i)
        #print(cone_bearings[i])

    for i in yellow_cones:
        cone_bearings_yellow[i] = find_angle(centre_point, i
                                             )

    # Now, we normalise the angles to start at 0 at the starting cone, then ascend anti-clockwise
    # Formula: angle += [(360 - start_angle) + angle] % 360
    for i in cone_bearings_blue:
        cone_bearings_blue[i] = math.fmod(((360 - start_angle_blue) + cone_bearings_blue[i]), float(360))
        #print(cone_bearings_blue[i])

    for i in cone_bearings_yellow:
        cone_bearings_yellow[i] = math.fmod(((360 - start_angle_yellow) + cone_bearings_yellow[i]), float(360))

    # Using the centre_point, we go around anti-clockwise and find a path for the innermost cones
    # Each of these cones are then matched up with their matching cone on the other side of the track
    # The following solution will only work for a circular track!
    cone_bearings_blue = {k: v for k, v in sorted(cone_bearings_blue.items(), key=lambda item: item[1])}
    cone_bearings_yellow = {k: v for k, v in sorted(cone_bearings_yellow.items(), key=lambda item: item[1])}

    dummy_blue = []
    dummy_yellow = []

    for i in cone_bearings_blue:
        # Check if this one is the start cone (angle = 0)
        if cone_bearings_blue[i] == 0:
            continue
        else:
            dummy_blue.append(i)

    for i in cone_bearings_yellow:
        # Check if this one is the start cone (angle = 0)
        if cone_bearings_yellow[i] == 0:
            continue
        else:
            dummy_yellow.append(i)

    for i in range(len(dummy_blue)):
        sorted_points_pair[i + 1] = (dummy_blue[i], dummy_yellow[i])

    sorted_listx = []
    sorted_listy = []
    sorted_colours = []

    # Now we put all the items into lists so it can be interpreted by our previous functions
    for i in sorted_points_pair:
        sorted_listx.append(sorted_points_pair[i][0].get_x())
        sorted_listy.append(sorted_points_pair[i][0].get_y())
        sorted_colours.append(sorted_points_pair[i][0].get_colour())

        sorted_listx.append(sorted_points_pair[i][1].get_x())
        sorted_listy.append(sorted_points_pair[i][1].get_y())
        sorted_colours.append(sorted_points_pair[i][1].get_colour())

    return sorted_listx, sorted_listy, sorted_colours


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
        cones[i] = (Point(listx[i * 2], listy[i * 2], colours[i * 2]),
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

    (xList, yList, colourList) = get_point_list(midpoints)

    # Plot out existing tracks
    plt.scatter(CX1, CY1, c=CC1)

    # Plot our midpoints
    plt.plot(xList, yList, '-o')
    plt.axis('equal')
    plt.show()


def task2():
    """ Task 2 """

    (xlist, ylist, colourlist) = sort_points(CX2, CY2, CC2)

    cones = get_point_pairs(xlist, ylist, colourlist)

    midpoints = get_track_midpoints(cones)

    (x_mids, y_mids, colour_mids) = get_point_list(midpoints)

    plt.scatter(CX2, CY2, c=CC2)

    plt.plot(x_mids, y_mids, '-o')
    plt.axis('equal')
    plt.show()

##------------------------------------------------------------------------------


def curve():
    """ "Racing curve" extension task"""


    x = np.linspace(0, 2*np.pi, 100)
    #fig = plt.figure()
    plt.polar(x, np.sin(x))
    plt.scatter(CX1, CY1, c=CC1)
    plt.show()


if __name__ == '__main__':

    #task1()
    #task2()
    curve()


# -------------------------------------------------------------------------------
