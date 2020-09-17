##   UQR DV New Member pathFinding task
##   code written by: Mitchell Anderson
##                    mitchell.anderson@uqracing.com
##   code mangled by: Alexander FitzGerald
##                    xanderfitzg2000@gmail.com


##------------------------------Import Statements-------------------------------
import collections

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

# ----- Constants -----
# Event classifications
TURN_START = 0
TURN_STOP = 1
THROTTLE = 2
BRAKE = 3
SHIFT_LEFT = 4
SHIFT_RIGHT = 5
TARGET = 6
FINISH_LINE = 7
HAIRPIN_ENTER = 8
HAIRPIN_APEX = 9
HAIRPIN_EXIT = 10
TURN_PREPARE = 11
STEER = 12

# Turn classifications
RIGHT_ANGLE = 0
DOUBLE_APEX = 1
HAIRPIN = 2
S_CURVE = 3
INCREASING_RADIUS = 4
DECREASING_RADIUS = 5


class TargetQueue:
    """
    Class for a queue of track targets, that we can ripple add to.
    """
    def __init__(self):
        """
        Constructor for TargetQueue class. This constructor takes no arguments
        """
        self.target_dict = {}
        self.queue_length = 0

    def add_target(self, index, target, classification=None):
        """
        Add a target to the queue at a given index. Targets after the given index are shifted up the queue by one

        :param index: Index of this target
        :param target: Point object as a target
        :param classification: Target action classification (OPTIONAL)
        :return: True if successfully added, False if invalid index given
        """
        if index == self.queue_length:
            # Add the item to the end of the queue
            self.target_dict[self.queue_length] = (target, classification)
            self.queue_length += 1
        elif index < self.queue_length:
            # Add the item into the middle of the queue
            for i in reversed(range(index + 1, self.queue_length + 1)):
                # Shift all proceeding elements to the right by one
                self.target_dict[i] = self.target_dict[i - 1]
            # Then, add the element
            self.target_dict[index] = (target, classification)
            self.queue_length += 1
            pass
        else:
            # Index out of bounds
            return False

    def get_queue_size(self):
        """
        Returns the queue size
        :return: Queue size
        """
        return self.queue_length

    def append_target(self, target, classification=None):
        """
        Append a target to the end of the queue

        :param target: Point object as a target
        :param classification: Target action classification (OPTIONAL)
        """
        self.add_target(self.queue_length, target, classification)

    def pop_target(self):
        """
        Pops the target at the top of the queue (lowest index)
        :return: (target (Point), classification), or None if the queue is empty
        """
        if self.queue_length > 0:
            if self.queue_length == 1:
                self.queue_length = 0
                return self.target_dict.pop(0)
            else:
                retval = self.target_dict[0]
                for i in range(0, self.queue_length - 1):
                    self.target_dict[i] = self.target_dict[i + 1]
                self.target_dict.pop(self.queue_length - 1)
                self.queue_length -= 1
                return retval
        else:
            return None

    def peek_target(self, index):
        """
        Returns the value at a given index. Does not remove it.
        This also supports negative integers for returning items at the end of the list, to be more Pythonic

        :param index: Index of target (negative numbers permitted for returning from end of list)
        :return: Target at that index. Returns none if index is invalid
        """
        if index < 0:
            if (index * -1) <= self.queue_length:
                # Valid negative index, start from the other side of the list (i.e. index -1 gives the last element)
                return self.target_dict[self.queue_length + index][0]
            else:
                return
        else :
            if index < self.queue_length:
                # Valid positive index
                return self.target_dict[index][0]
            else:
                return


class Waypoint:
    """
    Class for physical waypoints on a track, as a line across the width of the track that
    the car is guaranteed to cross

    A waypoint is directly associated with an Event that can be triggered when the waypoint is crossed
    """
    def __init__(self, midpoint, inner_bounds=None, outer_bounds=None, event=None, midpoints=None):
        """
        Constructor for waypoint class
        :param midpoint: Point in the middle of the track where the waypoint is based
        :param inner_bounds: List of Points corresponding to the inside of the track
        :param outer_bounds: List of Points corresponding to the outside of the track

        :param event: Event triggered by this waypoint (OPTIONAL)
        """
        self.midpoint = midpoint
        self.inner_bounds = inner_bounds
        self.outer_bounds = outer_bounds
        #self.point1 = None
        #self.point2 = None
        self.midpoint_bounds = None
        self.crossed = False

        if event is not None:
            self.event = event

        if midpoints is not None:
            self.midpoints = midpoints

        # Scaling line from one cone pair to the next
        found_line = False
        epsilon = 0.001
        steps = 300
        for i in range(len(inner_bounds)):
            # Prevent unnecessary further looping once we find the line
            if found_line:
                break

            point1_inner = inner_bounds[i]
            point2_inner = inner_bounds[i + 1] if i < (len(inner_bounds) - 1) else inner_bounds[0]
            point1_outer = outer_bounds[i]
            point2_outer = outer_bounds[i + 1] if i < (len(inner_bounds) - 1) else outer_bounds[0]

            delta_x_inner = point2_inner.get_x() - point1_inner.get_x()
            delta_y_inner = point2_inner.get_y() - point1_inner.get_y()
            delta_x_outer = point2_outer.get_x() - point1_outer.get_x()
            delta_y_outer = point2_outer.get_y() - point1_outer.get_y()

            # Try our initial search with coarse step to find the sector (more efficient than searching the entire
            # track with a fine step)
            for j in range(steps):
                multiplicand = j * (1 / steps)
                inner_point = Point(point1_inner.get_x() + (multiplicand * delta_x_inner),
                                    point1_inner.get_y() + (multiplicand * delta_y_inner))
                outer_point = Point(point1_outer.get_x() + (multiplicand * delta_x_outer),
                                    point1_outer.get_y() + (multiplicand * delta_y_outer))

                # Now we have the line, we want to determine the distance from the points to the waypoint's origin Point
                # We convert the Points to numpy-compatible arrays
                p_inner = np.array([inner_point.get_x(), inner_point.get_y()])
                p_outer = np.array([outer_point.get_x(), outer_point.get_y()])
                p_origin = np.array([self.midpoint.get_x(), self.midpoint.get_y()])

                # Use vector cross products to find the distance from the waypoint's origin point to the line
                dist = np.linalg.norm(np.cross(p_outer - p_inner, p_inner - p_origin)) / \
                       np.linalg.norm(p_outer - p_inner)

                max_distance_from_pair = 1
                if dist < epsilon:
                    dist1 = find_distance(self.midpoint, inner_point)
                    dist2 = find_distance(self.midpoint, outer_point)

                    if (dist1 <= max_distance_from_pair) and (dist2 <= max_distance_from_pair):
                        self.midpoint_bounds = (inner_point, outer_point)
                        found_line = True
                        plt.plot([inner_point.get_x(), outer_point.get_x()],
                                 [inner_point.get_y(), outer_point.get_y()], c='black')
                        break
                    else:
                        # We got a dodgy result from the cross product
                        # This occurs when we match with a waypoint line on the other side of the track
                        pass

    def get_waypoint_bounds(self):
        """
        Get the bounds of the waypoint line
        :return: Tuple: The bounds of the waypoint line
        """
        return self.midpoint_bounds

    def is_crossed(self):
        """
        Determines if this Waypoint has already been crossed
        :return: True if crossed, False if not yet crossed
        """
        return self.crossed

    def waypoint_distance(self, point):
        """
        Gets the distance from this Waypoint to a given point (the dodgy way!)
        :param point: A Point to measure to
        :return: Distance from the Point to the closest part of this Waypoint
        """

        local_dx = self.midpoint_bounds[1].get_x() - self.midpoint_bounds[0].get_x()
        local_dy = self.midpoint_bounds[1].get_y() - self.midpoint_bounds[0].get_y()
        point_span = []
        num_steps = 100
        for i in range(num_steps):
            multiplicand = i * (1 / num_steps)
            point_span.append(Point(self.midpoint_bounds[0].get_x() + (multiplicand * local_dx),
                                    self.midpoint_bounds[0].get_y() + (multiplicand * local_dy)))

        closest_point = None
        closest_distance = None
        for i in point_span:
            local_dist = find_distance(point, i)
            if closest_distance is None:
                closest_distance = local_dist
                closest_point = i
            else:
                if local_dist < closest_distance:
                    closest_distance = local_dist
                    closest_point = i

        return closest_distance


class Event:
    """
    Class for representing an event that happens on the track. Currently events include:
        TURN_START   (value: direction)
        TURN_STOP    (value: None)
        THROTTLE     (value: Amount)
        BRAKE        (value: Amount)
    """
    def __init__(self, event_type, value, time=None, location=None):
        """
        Constructor for the Event class

        :param event_type  : Event Type
        :param value : Value associated with event
        :param time  : Event time (along linspace)
        :param location : Event location as a Point
        """

        self.event_type = event_type
        self.value = value

        if time is not None:
            self.time = time

        if location is not None:
            self.location = location

    def get_event_type(self):
        """
        Get the type of this event
        :return: int: Type of event
        """
        return self.event_type

    def get_value(self):
        """
        Get the value of this event
        :return: int/float : Value associated with this event
        """
        return self.value

    def get_time(self):
        """
        Returns the event's time
        :return: t: Time of this event
        """
        return self.time

    def print_event(self):
        """
        Prints this event
        """
        print(str(self.event_type) + " Event @ t=" + str(self.time) + ", value=" + str(self.value))


class RacingCurve:
    """
    Class representing a curve on a track (typically taken by a vehicle)
    """
    def __init__(self, start, midpoints=None, inner_bounds=None, outer_bounds=None, ls=None, end_time=None,
                 starting_angle=None, events=None):
        """
        Class constructor, representing a curve on a track (typically taken by a vehicle)

        :param start : Point: start point

        :param midpoints : Dictionary of track midpoints (OPTIONAL)
        :param inner_bounds : List of Points - Track's inner bounds (OPTIONAL)
        :param outer_bounds : List of Points - Track's outer bounds (OPTIONAL)
        :param ls : Linspace - Linear space for indexing/parametrising curve (OPTIONAL)
        :param end_time : End time (in linspace) of the track (OPTIONAL)
        :param starting_angle : Angle the car starts driving (degrees from 0 - 360, with respect to the +x axis)
        :param events : List of Event objects (OPTIONAL)

        """
        self.start = start

        self.MAX_SPEED = 0.05

        # Point for storing car's current position
        self.position = start

        # Car's direction (0 - 360 degrees with respect to +x axis)
        self.direction = 0.0

        # Car's current "turning effort". 1 for max right, -1 for max left
        self.turn = 0.0

        # Car's steering sensitivity
        self.turn_sensitivity = 0.0005

        # Car's current speed (in direction of motion)
        self.speed = 0.0

        # Simulation end point (in time/linspace)
        self.end = 0

        # Throttle amount (acceleration)
        self.throttle = 0.0

        # Throttle sensitivity
        self.throttle_sensitivity = 0.005

        # Braking amount (deceleration)
        self.brake = 0.0

        # Brake sensitivity
        self.brake_sensitivity = 0.001

        # {t: (Point, direction, speed)}
        self.path = {}

        # OPTIONAL VALUES
        if midpoints is not None:
            self.midpoints = midpoints

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
            self.ls = np.linspace(0, 999, 1000)

        if end_time is not None:
            self.end_time = end_time
        else:
            self.step_size = 0

        if starting_angle is not None:
            self.starting_angle = starting_angle
        else:
            self.starting_angle = 0

        if events is not None:
            self.events = events
        else:
            self.events = []

    def get_path(self, t_start=None, t_end=None):
        """
        Finds a path with a specified start and end time
        
        :param t_start : Starting time (default value = 0) (OPTIONAL)
        :param t_end : End time (OPTIONAL)
        :return : Dict {time(in linspace): Point}
        """
        if t_start is None:
            t_start = 0

        # Mark out important events
        # TESTING - mark out midpoints[3]
        #dummy_waypoint = Waypoint(Point(3.5, 0.7), inner_bounds=self.inner_bounds, outer_bounds=self.outer_bounds,
        #                          event=Event(TURN_START, value=0), midpoints=self.midpoints)


        #plt.plot([dummy_waypoint.get_waypoint_bounds()[0].get_x(), dummy_waypoint.get_waypoint_bounds()[1].get_x()],
        #         [dummy_waypoint.get_waypoint_bounds()[0].get_y(), dummy_waypoint.get_waypoint_bounds()[1].get_y()])

        # Pass 1: Detecting turns
        # Follow the track midpoints around and detect changes in angle indicative of turns
        d_theta = []
        headings = []
        previous_theta = 0
        for i in self.midpoints:
            #self.midpoints[i]
            if i == 0:
                headings.append(0.0)
                d_theta.append(0.0)
            else:
                # Find current angle, and subtract previous angle
                current_heading = find_angle(self.midpoints[i - 1], self.midpoints[i])
                previous_heading = headings[i - 1]

                heading_change = current_heading - previous_heading

                dummy456 = 69

                headings.append(current_heading)
                d_theta.append(heading_change)
        d_theta.append((360 + headings[0]) - headings[-1])

        # Now, we detect the type of turn. We can determine the angle of the turn by following the turn
        # Looking for steep turn
        turns = {}
        in_turn = False
        nth_turn = -1
        for i in range(len(d_theta)):
            # If our current turning angle is high enough to count as a turn
            if d_theta[i] > 30.0:
                if in_turn is False:
                    # We have just entered a turn
                    nth_turn += 1
                    turns[nth_turn] = (self.midpoints[i], [], i)
                in_turn = True
            else:
                in_turn = False
            if in_turn:
                turns[nth_turn][1].append(d_theta[i])

        turn_classes = {}
        # Now we go through all the turns and classify them
        for i in turns:
            # 1st, check all turns are in the same direction:
            if all(item >= 0 for item in turns[i][1]) or all(item < 0 for item in turns[i][1]):
                # All the turns are in the same direction
                # Sub-classify turns
                angle_sum = sum(turns[i][1])
                if 70 < angle_sum < 120:
                    # This is a right-angle turn
                    turn_classes[i] = (RIGHT_ANGLE, turns[i][0])
                elif 150 < angle_sum < 200:
                    # This is a u-turn
                    # Now, we must determine if this is a double apex or a hairpin turn
                    # Check if each turn change is greater than than 35 degrees
                    if all(item >= 35 for item in turns[i][1]):
                        # We have a hairpin
                        turn_classes[i] = (HAIRPIN, turns[i][0])
                    else:
                        # We have a double-apex
                        turn_classes[i] = (DOUBLE_APEX, turns[i][0])
                # Other classifications can be written in here, haven't gotten around to it
            else:
                # We have an S-shaped curve
                turn_classes[i] = (S_CURVE, turns[i][0])

        # Now we go through and decide what to do for each waypoint
        raw_queue = TargetQueue()
        for i in turn_classes:
            if turn_classes[i][0] == RIGHT_ANGLE:
                # Process right-angle turn
                pass
            elif turn_classes[i][0] == DOUBLE_APEX:
                # Process double-apex turn
                pass
            elif turn_classes[i][0] == HAIRPIN:
                # Process hairpin turn

                # First, we need to get the 2 points that correspond to the first
                # and last inner cones associated with the turn.
                # Then, we draw a line from those 2 to the edges of the track
                # to find the waypoints for the racing curve
                turn_highest_index = (turns[i][2] - 2) + (len(turns[i][1]))
                temp_length = len(self.inner_bounds)

                point1 = self.inner_bounds[turns[i][2] - 1]

                # Checking if the last point of the turn is the
                if turn_highest_index >= temp_length:
                    # The last point is actually at the start line
                    point2 = self.inner_bounds[0]
                else:
                    point2 = self.inner_bounds[turn_highest_index]

                # Next, we find the turn enter and exit points.
                # A conventional hairpin turn entry point starts in line with the track turn apex, and on he opposite
                # side of the track. The exit is similarly in line with the apex of the track turn, but as close in
                # to the track turn as possible. i.e. we start the the turn on the outside and move to the inside
                # To find the outside edge entry point, we need to find the outside interval that intersects the line
                # described by the 2 inner-edge points that correspond to the start and the end of the turn.
                # The apex line (drawn from side to side of the track, tangential to the apex of the track turn inside)
                # describes a line, Draw a line from point2 to point1 to determine direction. Then we draw a line from
                # point1 in the point2 -> point1 direction, sufficiently long enough to clip the edge of the track
                # Find unit vectors from point2 to point1
                x_unit = point1.get_x() - point2.get_x()
                y_unit = point1.get_y() - point2.get_y()
                unit_mag = math.sqrt((x_unit ** 2) + (y_unit ** 2))
                x_unit *= (1 / unit_mag)
                y_unit *= (1 / unit_mag)

                # Now we determine a point in line with these points and off the track
                intersector_length = 10
                off_point = Point(point1.get_x() + intersector_length * x_unit,
                                  point1.get_y() + intersector_length * y_unit)

                entry_point = None
                apex_point = None
                exit_point = None

                # First we look for the outer turn point
                for j in range(len(self.outer_bounds)):
                    # Draw a segment between this outer cone and the next outer cone
                    outer_point1 = self.outer_bounds[j]
                    outer_point2 = self.outer_bounds[j + 1] if j + 1 < len(self.outer_bounds) else self.outer_bounds[0]
                    result = get_segment_intersection(point1, off_point, outer_point1, outer_point2)
                    if result is not None:
                        # We found the intersection
                        entry_point = result

                        # We also might move it in a tiny bit so it's not *literally* on the edge of the track
                        new_x = entry_point.get_x() - (0.1 * x_unit)
                        new_y = entry_point.get_y() - (0.1 * y_unit)
                        entry_point.set_x(new_x)
                        entry_point.set_y(new_y)
                        plt.scatter([entry_point.get_x()], [entry_point.get_y()], marker='x', c='red')
                        break
                    else:
                        # This is not the intersection interval. Keep looking
                        continue

                # Next, we look for the turn apex
                if len(turns[i][1]) % 2 == 1:
                    # If we have a turns list with odd number, we can take the middle point as the apex
                    apex_point = self.midpoints[((len(turns[i][1]) - 1) / 2) + (turns[i][2] - 1)]
                    pass
                else:
                    # We have to determine the middle point for the apex
                    index_first = int((((len(turns[i][1])) / 2) - 1) + (turns[i][2] - 1))
                    index_last = int(((len(turns[i][1])) / 2) + (turns[i][2] - 1))

                    inner1 = self.inner_bounds[index_first]
                    inner2 = self.inner_bounds[index_last] if index_last < turn_highest_index else self.inner_bounds[0]

                    outer1 = self.outer_bounds[index_first]
                    outer2 = self.outer_bounds[index_last] if index_last < turn_highest_index else self.outer_bounds[0]

                    inner_mid = cone_pair_midpoint(inner1, inner2)
                    outer_mid = cone_pair_midpoint(outer1, outer2)
                    apex_point = cone_pair_midpoint(inner_mid, outer_mid)

                plt.scatter([apex_point.get_x()], [apex_point.get_y()], marker='x', c='red')

                # Finally, we get the exit point
                exit_point = Point(point2.get_x() - (0.1 * x_unit), point2.get_y() - (0.1 * y_unit))

                plt.scatter([exit_point.get_x()], [exit_point.get_y()], marker='x', c='red')

                # Now that we have the three points, we add these to a queue
                # And then these queues are added to a master queue
                sub_queue = TargetQueue()
                sub_queue.append_target(entry_point, classification=HAIRPIN_ENTER)
                sub_queue.append_target(apex_point, classification=HAIRPIN_APEX)
                sub_queue.append_target(exit_point, classification=HAIRPIN_EXIT)

                raw_queue.append_target(sub_queue)

                dummy456789 = 42

            elif turn_classes[i][0] == S_CURVE:
                # Process S-curve
                pass
            elif turn_classes[i][0] == INCREASING_RADIUS:
                # Process increasing radius turn
                pass
            elif turn_classes[i][0] == DECREASING_RADIUS:
                # Process decreasing radius turn
                pass
            else:
                pass

        # Now we've got a queue of queues of targets. We now need to work out preparations for each event
        # Finding the index of the closest midpoint

        # Iterate over the "master" queue
        for i in range(raw_queue.get_queue_size()):
            this_queue = raw_queue.peek_target(i)

            # Then get the first item in the queue
            this_point = None
            if isinstance(this_queue, TargetQueue):
                this_point = this_queue.peek_target(0)
            else:
                print("not a queue")

            # Now we have the first point of this sequence, find its closest midpoint
            closest_distance = None
            closest_index = None
            for j in range(len(self.midpoints)):
                if closest_index is None:
                    # For the first run through
                    closest_index = j
                    closest_distance = find_distance(this_point, self.midpoints[j])
                else:
                    # For following runs through
                    local_distance = find_distance(this_point, self.midpoints[j])
                    if local_distance < closest_distance:
                        # We have found an even closer midpoint
                        closest_index = j
                        closest_distance = local_distance

            midpoint_ahead = self.midpoints[closest_index]
            midpoint_behind = self.midpoints[closest_index - 1]

            # Then find unit vectors from midpoint_ahead to midpoint_behind (opposite direction to track driving)
            turn_leading_unit_x = midpoint_behind.get_x() - midpoint_ahead.get_x()
            turn_leading_unit_y = midpoint_behind.get_y() - midpoint_ahead.get_y()
            turn_leading_mag = math.sqrt((turn_leading_unit_x ** 2) + (turn_leading_unit_y ** 2))
            turn_leading_unit_x *= (1 / turn_leading_mag)
            turn_leading_unit_y *= (1 / turn_leading_mag)
            turn_prep_distance = 1

            # Next, we need to place a point a bit before the turn to prepare for it and line up
            prep_target = Point(this_point.get_x() + turn_leading_unit_x,
                                 this_point.get_y() + turn_leading_unit_y)

            plt.scatter([prep_target.get_x()], [prep_target.get_y()], marker='o', c='pink', s=200)
            plt.scatter([prep_target.get_x()], [prep_target.get_y()], marker='$HP$', c='red', s=150)


            this_queue.add_target(0, prep_target, classification=TURN_PREPARE)

            meme435 = 35

        dummy456 = 123

        track_target_queue = TargetQueue()
        for i in range(raw_queue.get_queue_size()):
            popped = raw_queue.pop_target()
            for j in range(popped[0].get_queue_size()):
                last_target = popped[0].pop_target()
                track_target_queue.append_target(last_target[0], classification=last_target[1])
                emme13 = 234


        path_points = {}
        current_events = []
        finish_line = Waypoint(self.midpoints[0], self.inner_bounds, self.outer_bounds, midpoints=self.midpoints)

        # Go through each t value in the linspace
        for t in range(t_start, t_end):
            # First, check if we've just finished the track
            distance_to_finish_line = finish_line.waypoint_distance(self.position)

            # Make sure we start the finish line triggering once we get in a bit, so it doesn't trigger as soon
            # as we start
            if t > 10:
                if distance_to_finish_line < 0.05:
                    # We have finished the race. Stop the simulation
                    break

            # 1: Determine angle and velocity
            self.speed += (self.throttle * self.throttle_sensitivity) - \
                          (self.brake * self.brake_sensitivity)

            # Limit to a simulated max speed
            if self.speed > self.MAX_SPEED:
                self.speed = self.MAX_SPEED

            # 2: Determine new position
            # Determine change in heading (from steering)
            # This is a constant, mu * g
            turn_radius_constant = 20

            # Calculate max turn radius = v^2 / mu * g
            max_turn_radius = (self.speed ** 2) / turn_radius_constant
            (x_unit, y_unit) = angle_to_units(self.direction)
            x_unit *= self.speed
            y_unit *= self.speed

            delta_position = math.sqrt((x_unit ** 2) + (y_unit ** 2))

            # Avoid divide by 0
            if max_turn_radius == 0:
                max_turn_degrees = 0
            else:
                max_turn_degrees = math.degrees(delta_position / max_turn_radius)

            self.direction = (self.direction + ((self.turn * -1 * self.turn_sensitivity) * max_turn_degrees)) % 360

            self.position = Point(self.position.get_x() + x_unit,
                                  self.position.get_y() + y_unit)

            path_points[t] = self.position

            # 3: Check for events here
            if self.events is not None:
                current_events = return_event_here(t, self.events)
                if current_events is not None:
                    for j in current_events:
                        #j.print_event()

                        # Switch on event type
                        if j.get_event_type() == TURN_START:
                            # Process start of a turn
                            pass
                        if j.get_event_type() == TURN_STOP:
                            # Process end of a turn
                            pass
                        if j.get_event_type() == THROTTLE:
                            # Process throttle engage
                            self.throttle = j.get_value()
                        if j.get_event_type() == BRAKE:
                            # Process brake engage
                            self.brake = j.get_value()
                            if self.brake != 0:
                                self.throttle = 0
                        if j.get_event_type() == STEER:
                            self.turn = j.get_value()

            # 4: Check for important events a bit into the near future
            # Get the next queued target
            next_queued = track_target_queue.peek_target(0)
            # Check for turning targets
            if track_target_queue.get_queue_size() == 0:
                # No more events to process
                continue
            else:
                if track_target_queue.target_dict[0][1] == TURN_PREPARE:
                    if find_distance(self.position, next_queued) < 0.05:
                        # We've already basically hit the point, move on to the next one
                        track_target_queue.pop_target()
                        continue

                    # Look for the next item in the queue, so we can arrive already lined up with it
                    next_target = track_target_queue.peek_target(1)

                    # Get direction angle to the next target
                    new_bearing = find_angle(next_queued, next_target)
                    # So we need to be coming into this point with that angle
                    (this_x_unit, this_y_unit) = angle_to_units(new_bearing)

                    # Turn towards a point on that line
                    # We need to make a list of points that occur on the line, then find the closest point
                    spaced_points = []
                    other_side = Point(next_queued.get_x() - (10 * this_x_unit), next_queued.get_y() - (10 * this_y_unit))

                    # Draw a whole bunch of evenly-spaced points between these 2 points
                    dx = next_queued.get_x() - other_side.get_x()
                    dy = next_queued.get_y() - other_side.get_y()
                    numsteps = 100
                    for teeny in range(numsteps):
                        mx = teeny * (1 / numsteps)
                        spaced_points.append(Point(other_side.get_x() + (mx * dx), other_side.get_y() + (mx * dy)))

                    closest_point = None
                    other_closest_distance = None
                    for tiny in spaced_points:
                        if other_closest_distance is None:
                            closest_point = tiny
                            other_closest_distance = find_distance(tiny, self.position)
                        else:
                            this_dist = find_distance(tiny, self.position)
                            if this_dist < other_closest_distance:
                                closest_point = tiny
                                other_closest_distance = this_dist
                    new_goal = cone_pair_midpoint(closest_point, next_queued)

                    # Now we need to just point towards "new_goal". Find the angle change required to do it:
                    required_d_theta = find_angle(self.position, new_goal) - self.direction

                    # Determine needed steering direction

                    steer_required = None
                    if (required_d_theta > 355 or required_d_theta < 5):
                        steer_required = 0
                    else:
                        if required_d_theta > 180:
                            # Need to turn right
                            steer_required = 0.4
                        else:
                            steer_required = -0.4

                    self.events.append(Event(STEER, steer_required, time=(t + 1)))
                    #self.turn = steer_required

                elif track_target_queue.target_dict[0][1] == HAIRPIN_ENTER:
                    if find_distance(self.position, next_queued) < 0.05:
                        # We've already basically hit the point, move on to the next one
                        track_target_queue.pop_target()
                        continue

                    new_goal = next_queued

                    # Now we need to just point towards "new_goal". Find the angle change required to do it:
                    required_d_theta = (find_angle(self.position, new_goal) - self.direction) % 360

                    # Determine needed steering direction

                    steer_required = None
                    if (required_d_theta > 359 or required_d_theta < 1):
                        steer_required = 0
                    else:
                        if required_d_theta > 180:
                            # Need to turn right
                            steer_required = 0.4
                        else:
                            steer_required = -0.4

                    self.events.append(Event(STEER, steer_required, time=(t + 1)))
                    #self.turn = steer_required

                elif track_target_queue.target_dict[0][1] == HAIRPIN_APEX:
                    if find_distance(self.position, next_queued) < 0.05:
                        # We've already basically hit the point, move on to the next one
                        track_target_queue.pop_target()
                        continue
                    self.events.append(Event(BRAKE, 0.69, time=(t + 1)))

                    # Look for the next item in the queue, so we can arrive already lined up with it
                    next_target = track_target_queue.peek_target(1)

                    # Get direction angle to the next target
                    new_bearing = find_angle(next_queued, next_target)
                    # So we need to be coming into this point with that angle
                    (this_x_unit, this_y_unit) = angle_to_units(new_bearing)

                    # Turn towards a point on that line
                    # We need to make a list of points that occur on the line, then find the closest point
                    spaced_points = []
                    other_side = Point(next_queued.get_x() - (10 * this_x_unit), next_queued.get_y() - (10 * this_y_unit))

                    (my_unit_x, my_unit_y) = angle_to_units(self.direction)
                    very_me = Point(self.position.get_x() + (100 * my_unit_x),
                                    self.position.get_y() + (100 * my_unit_y))

                    intersect_goal = get_segment_intersection(next_queued, other_side, self.position, very_me)
                    if intersect_goal is None:
                        intersect_goal = next_queued
                    distance_to_goal = find_distance(self.position, intersect_goal)

                    dummy1235 = 345

                    new_dx = next_queued.get_x() - intersect_goal.get_x()
                    new_dy = next_queued.get_y() - intersect_goal.get_y()

                    if distance_to_goal > 1.8:
                        a_position = 0.25
                    else:
                        a_position = 0.16

                    new_goal = Point(intersect_goal.get_x() + (a_position * new_dx), intersect_goal.get_y() + (a_position * new_dy))
                    #new_goal = cone_pair_midpoint(intersect_goal, next_queued)

                    # Now we need to just point towards "new_goal". Find the angle change required to do it:
                    required_d_theta = find_angle(self.position, new_goal) - self.direction

                    # Determine needed steering direction

                    steer_required = None
                    if (required_d_theta > 355 or required_d_theta < 5):
                        steer_required = 0
                    else:
                        if required_d_theta > 180:
                            # Need to turn right
                            steer_required = 0.35
                        else:
                            steer_required = -0.35

                    self.events.append(Event(STEER, steer_required, time=(t + 1)))
                    asdf=1234

                elif track_target_queue.target_dict[0][1] == HAIRPIN_EXIT:
                    if find_distance(self.position, next_queued) < 0.05:
                        # We've already basically hit the point, move on to the next one
                        track_target_queue.pop_target()
                        continue
                    # Now we're out of the hairpin apex. off the brakes and pin the throttle
                    self.events.append(Event(BRAKE, 0, time=(t + 1)))
                    self.events.append(Event(THROTTLE, 1, time=(t + 1)))

                    new_goal = next_queued
                    #new_goal = cone_pair_midpoint(intersect_goal, next_queued)

                    # Now we need to just point towards "new_goal". Find the angle change required to do it:
                    required_d_theta = (find_angle(self.position, new_goal) - self.direction) % 360

                    # Determine needed steering direction

                    steer_required = None
                    if (required_d_theta > 355 or required_d_theta < 5):
                        steer_required = 0
                    else:
                        if required_d_theta > 180:
                            # Need to turn right
                            steer_required = 0.3
                        else:
                            steer_required = -0.3

                    self.events.append(Event(STEER, steer_required, time=(t + 1)))

                else:
                    # It's some other kind of turn
                    pass

        return path_points


class Point:
    """
    Class representing a point on the track

    :param x      : x coordinate of point
    :param y      : y coordinate of point
    :param colour : OPTIONAL - colour of point

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

    def set_x(self, new_x):
        self.x = new_x

    def set_y(self, new_y):
        self.y = new_y

    def set_colour(self, new_colour):
        self.colour = new_colour

    def compare_to(self, other):
        """
        Compares this point to another point, return true if it is at the same location, false otherwise

        :param other : Other Point
        :return : True if same location as other point, false otherwise
        """
        return True if ((other.get_x() == self.get_x()) and (other.get_y() == self.get_y())) else False


def get_segment_intersection(p0, p1, p2, p3):
    """
    Get the intersection between two line segments, with Points as line parameters
    Returns None if there is no intersection

    :param p0: Point 1, line 1
    :param p1: Point 2, line 1
    :param p2: Point 1, line 2
    :param p3: Point 2, line 2
    :return: Point of intersection if the line segments intersect,
             None if there is no intersection
    """
    p0_x = p0.get_x()
    p0_y = p0.get_y()
    p1_x = p1.get_x()
    p1_y = p1.get_y()
    p2_x = p2.get_x()
    p2_y = p2.get_y()
    p3_x = p3.get_x()
    p3_y = p3.get_y()

    s1_x = p1_x - p0_x
    s1_y = p1_y - p0_y
    s2_x = p3_x - p2_x
    s2_y = p3_y - p2_y

    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y)
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y)

    if 0 <= s <= 1 and 0 <= t <= 1:
        # Intersection found
        int_x = p0_x + (t * s1_x)
        int_y = p0_y + (t * s1_y)
        return Point(int_x, int_y)

    return None


def return_event_here(time, events):
    """
    Get a list of all the events at a given time
    :param time: Time in linspace
    :param events: List of events

    :return: List of events that occur at this time
    """
    event_list = []
    for i in events:
        if i.get_time() == time:
            event_list.append(i)

    return event_list


def angle_to_units(angle):
    """
    Converts an angle to unit vectors in the x and y directions
    :param angle : Angle (in degrees) from 0 - 360

    :return: Tuple: (x_unit, y_unit)
    """
    x_unit = math.cos(math.radians(angle))
    y_unit = math.sin(math.radians(angle))

    return x_unit, y_unit


def get_points_mean(points):
    """
    Get the middle point of a set of points

    :param points : List of points

    :return: Point: Middle point of all points (simple mean)
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

    :param list_x       : List of x coordinates
    :param list_y       : List of y coordinates
    :param list_colours : List of colours for each point

    :return: List of points
    """
    points = []
    for i in range(len(list_x)):
        points.append(Point(list_x[i], list_y[i], list_colours[i]))

    return points


def find_distance(origin, dest):
    """
    Find the (Euclidean) distance between 2 Points

    :param origin : Origin point (reference)
    :param dest   : Point to find distance to

    :return : Euclidean distance to point
    """
    x = abs(origin.get_x() - dest.get_x())
    y = abs(origin.get_y() - dest.get_y())
    return math.sqrt(math.pow(x, 2) + math.pow(y, 2))


def find_closest(origin, points):
    """
    Find the Point that is closest to a given Point

    :param origin : Origin (reference) point
    :param points : List of Points

    :return : Point: whichever point is closest
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

    :param origin : Origin point (reference)
    :param dest   : Destination point to measure to

    :return : Angle to destination point (from +x axis)
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

    :param point1 : First Point
    :param point2 : Second Point

    :return       : New Point object, at the midpoint
    """

    xval = (point1.get_x() + point2.get_x()) / 2
    yval = (point1.get_y() + point2.get_y()) / 2
    retpoint = Point(xval, yval)

    return retpoint


def sort_points(listx, listy, colours):
    """
    Given a list of x coords, y coords, and colours, sort the points
    ASSUMING THE TRACK IS CIRCULAR/OVAL, AND WE MAKE AN ANTI-CLOCKWISE PATH

    :param listx   : List of points' x coordinates
    :param listy   : List of points' y coordinates
    :param colours : List of points' colours

    :return : (sorted_x, sorted_y, sorted_colours):
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
        cone_bearings_yellow[i] = find_angle(centre_point, i)

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

    :param listx   : List of x coordinates
    :param listy   : List of y coordinates
    :param colours : List of cone colours

    :return : Dictionary with indices as keys, and Point tuples as entries
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
    :param points : Dictionary containing point pairs with indices
    @type points  : Dictionary := {index: (Point, Point)}

    :return       : Dictionary containing midpoints with indices
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

    :param points   : Dictionary mapping point indices to Point objects
    :return         : Tuple containing list of x coordinates, list of y coordinates, and list of colours
    """

    xList = []
    yList = []
    colourList = []

    for i in points:
        xList.append(points[i].get_x())
        yList.append(points[i].get_y())
        colourList.append(points[i].get_colour())

    return xList, yList, colourList


def test_print_dict(q):
    length = q.get_queue_size()
    if length <= 0:
        return
    else:
        result = ""
        for i in range(length):
            result += " [" + str(q.target_dict[i][0]) + "] "
        print(result)


def task1():
    """ Task 1 """

    # Store all the cone pairs as tuples in a dictionary, with its index as a key
    cones = get_point_pairs(CX1, CY1, CC1)

    # Store all the midpoints as dictionary entries, with its index as a key
    midpoints = get_track_midpoints(cones)

    (xList, yList, colourList) = get_point_list(midpoints)

    # Plot out existing tracks
    plt.scatter(CX1, CY1, c=CC1)

    blue_points = []
    yellow_points = []

    for i in cones:
        blue_points.append(cones[i][0])
        yellow_points.append(cones[i][1])

    inside_list_x = []
    inside_list_y = []
    outside_list_x = []
    outside_list_y = []

    for i in blue_points:
        inside_list_x.append(i.get_x())
        inside_list_y.append(i.get_y())
    inside_list_x.append(blue_points[0].get_x())
    inside_list_y.append(blue_points[0].get_y())

    for i in yellow_points:
        outside_list_x.append(i.get_x())
        outside_list_y.append(i.get_y())

    outside_list_x.append(yellow_points[0].get_x())
    outside_list_y.append(yellow_points[0].get_y())

    plt.plot(inside_list_x, inside_list_y)
    plt.plot(outside_list_x, outside_list_y)



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

    blue_points = []
    yellow_points = []

    for i in cones:
        blue_points.append(cones[i][0])
        yellow_points.append(cones[i][1])

    temp = blue_points[0]
    blue_points[0] = yellow_points[0]
    yellow_points[0] = temp

    inside_list_x = []
    inside_list_y = []
    outside_list_x = []
    outside_list_y = []

    for i in blue_points:
        inside_list_x.append(i.get_x())
        inside_list_y.append(i.get_y())
    inside_list_x.append(blue_points[0].get_x())
    inside_list_y.append(blue_points[0].get_y())

    for i in yellow_points:
        outside_list_x.append(i.get_x())
        outside_list_y.append(i.get_y())

    outside_list_x.append(yellow_points[0].get_x())
    outside_list_y.append(yellow_points[0].get_y())

    plt.plot(inside_list_x, inside_list_y)
    plt.plot(outside_list_x, outside_list_y)



    plt.scatter(CX2, CY2, c=CC2)

    plt.plot(x_mids, y_mids, '-o')
    plt.axis('equal')
    plt.show()

##------------------------------------------------------------------------------


def curve():
    """ "Racing curve" extension task"""

    cones = get_point_pairs(CX1, CY1, CC1)
    midpoints = get_track_midpoints(cones)

    blue_points = []
    yellow_points = []

    for i in cones:
        blue_points.append(cones[i][0])
        yellow_points.append(cones[i][1])

    # We start off by adding a throttle event at t=0
    initial_events = [Event(THROTTLE, 1.0, time=0)]
    racing_curve = RacingCurve(midpoints[0], midpoints=midpoints, starting_angle=0, events=initial_events,
                               inner_bounds=blue_points, outer_bounds=yellow_points)
    curve_points = racing_curve.get_path(t_start=0, t_end=250)
    curve_points_list = list(curve_points.values())

    (x_list, y_list, colour_list) = get_point_list(curve_points)

    (x_mids, y_mids, colour_mids) = get_point_list(midpoints)

    x = np.linspace(0, 2*np.pi, 100)

    inside_list_x = []
    inside_list_y = []
    outside_list_x = []
    outside_list_y = []

    for i in blue_points:
        inside_list_x.append(i.get_x())
        inside_list_y.append(i.get_y())
    inside_list_x.append(blue_points[0].get_x())
    inside_list_y.append(blue_points[0].get_y())

    for i in yellow_points:
        outside_list_x.append(i.get_x())
        outside_list_y.append(i.get_y())

    outside_list_x.append(yellow_points[0].get_x())
    outside_list_y.append(yellow_points[0].get_y())

    #plt.plot(x_mids, y_mids, '-o')
    plt.scatter(CX1, CY1, c=CC1)
    plt.plot(inside_list_x, inside_list_y)
    plt.plot(outside_list_x, outside_list_y)

    plt.plot(x_list, y_list, 'o-')
    plt.axis([-0.1, 5.1, -1, 3])
    plt.show()

    dummy123 = 420.69


if __name__ == '__main__':

    #task1()
    #task2()
    curve()


# -------------------------------------------------------------------------------
