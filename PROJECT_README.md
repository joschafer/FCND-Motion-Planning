## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This document provides the required writeup.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

These two scripts provide a combination of an A* planning algorithm implementation and a drone class that will follow the path from beginning to end.  These are each describe in more detail below.

`motion_planning.py` is the master script for creating a plan and executing a flight from the center of a set of obstacles read from a CSV to 10 units north and 10 units east of the start within the flight simulator.  There are two classes that are contained within the file as well as the main code which starts a connection to the simulator, instantiates the drone with the connection, and starts the flight.  

The first class (States) defined within `motion_planning.py` is an enumeration of the states that the drone may move through over the duration of the flight. These are used by the second class (MotionPlanning) which defines our drone and how it behaves as it responds to events from the simulator. The possible states and their meanings are described below.

Sequence | State | Description
--- | --- | ---
 1 | MANUAL | Starting and ending state
 2 | ARMING | Take control of drone in simulator
 3 | PLANNING | Plan path based on start and goal
 4 | TAKEOFF | Drone lifts off to desired altitude
 5 | WAYPOINT | Progressing along path.  Visited multiple times
 6 | LANDING | Drone descends to ground level
 7 | DISARMING | Release control of drone in simulator


The second class within `motion_planning.py` is the drone code that will respond to events and track the states of the drone.  Three specific events have callbacks registered that control the state transitions.  These are LOCAL_POSITION, LOCAL_VELOCITY, and STATE.  The STATE event is used for simple sequencing through drone states in which the drone is stationary and on the ground.  The LOCAL_VELOCITY event is used to determine that the drone has slowed enough (reached the ground) so that it may be safely disarmed.  The LOCAL_POSITION event is used most frequently and it provides verification that desired altitude is reached on takeoff, that each intermediate waypoint is reached, and that when the final waypoint is reached, landing may occur.    

The most interesting transition that occurs in the drone in the plan_path.  It takes the start and goal locations, which are simple in the default implementation, calls a_star to plan the path, and sets up the waypoints.

`planning_utils.py` provides several support classes and functions.  The first of these is the create_grid function which creates a grid based on the size of the data provided and populates it with the obstacles within the grid.  The grid is then used in the A* planning for determining the route.

Second in `planning_utils.py` is the Action enumeration which lists the possible moves that may be made within the grid, regardless of obstacle.  It also includes the cost of each move to help pick a less costly plan.

Third in `planning_utils.py` is the valid_actions function which narrows the possible unconstrained moves to those that are valid from the current location.  This eliminates obstacles and prevents attempts to move beyon the grid.

Fourth in  `planning_utils.py` is the a_star function which implements an A* search over the the grid for a valid and cost effective path between the start and the goal locations.  It performs these actions by doing a breadth first cost prioritized search of the possible moves in the grid.  As each location is visited, valid, unvisited moves are added to a cost prioritized queue to be searched next.  When the goal location is reached, the queueu processing is stopped and the final path that was traversed will be returned.  In the event that no path is found, a message is printed and an empty path is returned.

Lastly, in `planning_utils.py` a heuristic funtion is provided to provide an estimate of the cost from a position to the goal position.



### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

Code is included below from `motion_planning.py`.  I opened the file, read the first line, and split it to return lat0 and lon0.

    with open('colliders.csv') as f:
        first_line = f.readline().strip()
    (_, lat0, _, _, lon0) = first_line.replace(' ',',').split(',')


#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.


Code is included below from `motion_planning.py`.  To get the current local position, I grabbed the global positions and used the global_to_local from `udacidrone.frame_utils`,

    start_global_position =
        (self._longitude, self._latitude, self._altitude)
    start_local_position = global_to_local
        (start_global_position, self.global_home)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

Code is included below from motion.py.  The local position described in the previous step was used along with the side of the grid (via the offsets) to set a grid position where we are starting.

    grid_start = (-north_offset + int(start_local_position[0]),
                  -east_offset + int(start_local_position[1]))


#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

Code is included below from `motion_planning.py`.  The latitude and longitude were determined by flying around manually, grabbing the values from the simulator, and plugging them in to the code as constants.  They were then converted to the grid goal by using global_to_local and adding offsets as previously described.

    gl_grid_goal = (-122.395989, 37.795227,0)
    gl_grid_x_y = global_to_local(gl_grid_goal, self.global_home)
    grid_goal = (-north_offset + int(gl_grid_x_y[0]),
                 -east_offset + int(gl_grid_x_y[1]))


#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

Code is included below from `planning_utils.py`.  This implementation extended the existing A* implementation to add diagonal moves in the grid.  The first code snippet was used to add the new directions and costs to the Action enumeration.  This would make these allowable moves.  The second code snippet shows how invalid diagonal moves are removed from the set of possible moves to be tested.  It is worth noting that the boundary checks are kept in to prevent an attempt to check a diagonal move that is beyond any boundary.

    # Snippet 1

    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))

    # Snippet 2

    # NE obstacle
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    # NW obstacle
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    # SE obstacle
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)
    # SW obstacle
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)




#### 6. Cull waypoints
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.


Code to cull waypoints was used from a previous class exercise.  It was put in a separate file, `planning_utils_additions.py`, to allow replacement in the future if desired.  This code performs a collinearity test using a determinant.  The pruning code walks through the planned path and makes the collinearity check based on the current and next two points.  If the three points are collinear, then the "middle" point is removed from the planned path and the processing continues on with the next point in succession.


### Execute the flight
#### 1. Does it work?
This works.  Images below are the near the start location after takeoff and at the goal position after landing.

![Start Position](./misc/NearStartFlying.png)

![End Position](./misc/AtGoalLanded.png)

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.

# Extra Challenges: Real World Planning

No extra challenges were completed but minor improvements were added to prevent calculation of a path if the start and goal locations were identical as well exiting immediately if no valid path could be found.
