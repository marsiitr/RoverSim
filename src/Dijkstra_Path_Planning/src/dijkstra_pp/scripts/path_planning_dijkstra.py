#!/usr/bin/env python

import rospy
from path_planning_msg.srv import pp_msg, pp_msgResponse
from geometry_msgs.msg._Twist import Twist

# Fn to find neighbours
# Params: Index of cell of which neighbours are to be found,
#         Width of costmap, Height of costmap, Costmap


def find_neighbours(index, width, height, costmap):

    rospy.logdebug("Finding neighbours for (" + str((index + 1 - ((index + 1) %
                   width)) / width) + ", " + str((index + 1) % width) + ")")

    # Empty list for neighbours
    neighbours = []

    # The cost above which neihbours will not be counted
    lethal_cost = 1

    # Upper cell
    upper = index - width  # costmap passed on by plugin is 1d
    if upper > 0:
        if costmap[upper] < lethal_cost:
            neighbours.append(upper)

    # lower cell
    lower = index + width
    if lower < width * height:
        if costmap[lower] < lethal_cost:
            neighbours.append(lower)

    # right cell
    right = index + 1
    if right % width != width - 1:
        if costmap[right] < lethal_cost:
            neighbours.append(right)

    # left cell
    left = index - 1
    if left % width != 0:
        if costmap[left] < lethal_cost:
            neighbours.append(left)

    # rospy.logdebug("Neighbours found! Returning.......")

    return neighbours

# Fn to find the path using Dijkstra's algo
# Params: Start index, goal index,
#         Height and Width of coatmap, Costmap


def dijkstra(start, goal, width, height, costmap):
    # Open list - Contains the list of node that have been found as neighbours but they have not been processed
    # Closed list - Contains the list of nodes that have been processsed completely

    rospy.logdebug("Starting Dijkstra Path planning..... Initializing vars")
    rospy.loginfo("Starting Dijkstra Path planning.....")

    # Define all the required variables
    open_list = []
    closed_list = []
    parents = {}
    g_costs = {}
    path = []
    path_found = False

    rospy.logdebug("Variables initialised")

    # Set the cost of starting node as 0 and add it to the open list
    g_costs[start] = 0
    open_list.append([start, g_costs[start]])

    # Loop till the open list is empty
    while open_list.__len__ != 0:

        # Sort the open list according to g_cost and set the node with lowest g_cost as current node
        open_list.sort(key=lambda x: x[1])
        current_node = open_list.pop(0)[0]
        closed_list.append(current_node)

        # Check if the selected node is the goal
        if current_node == goal:
            path_found = True
            rospy.logdebug("Path found!")
            rospy.loginfo("Path found!")
            break

        # Get the neighbours of the current node
        neighbours = find_neighbours(current_node, width, height, costmap)
        # Iterate through every neighbour of current node
        for neighbour_index in neighbours:
            # If neighbour is in closed list then skip the neighbour
            if neighbour_index in closed_list:
                continue

            # Calculate the g_cost of neighbour with current node as parent
            g_cost = g_costs[current_node] + 1

            # Check if neighbour is in open list
            neighbour_in_open_list = False
            for index, node_list in enumerate(open_list):
                if node_list[0] == neighbour_index:
                    neighbour_in_open_list = True
                    break

            if neighbour_in_open_list:
                # If neighbour is in open list and calculated g_cost with current node is less
                # than the g_cost stated in open list, update the g_cost and parent according to current node
                if g_cost < g_costs[neighbour_index]:
                    g_costs[neighbour_index] = g_cost
                    parents[neighbour_index] = current_node
                    open_list[index] = [neighbour_index, g_cost]

            else:
                # If neighbour is not in open list, add this neighbour to open list
                g_costs[neighbour_index] = g_cost
                parents[neighbour_index] = current_node
                open_list.append([neighbour_index, g_cost])

    if path_found:
        # Set the path list
        node = goal
        rospy.loginfo("Generating path......")
        rospy.logdebug("Generating path......")
        while node != start:
            path.append(node)
            node = parents[node]
        path.append(start)

        # Reverse the path list (nodes were added in backwards order)
        rospy.logdebug("Reversing path list.....")
        path = path.reverse()

    else:
        rospy.logwarn("Path not found!")

    rospy.logdebug("Exiting Path planning algo........")
    return path


# ROS server to return path
def make_plan(req):
    costmap = req.costmap
    width = req.width
    height = req.height
    start = req.start
    goal = req.goal

    path = dijkstra(start, goal, width, height, costmap)

    if path.__len__ == 0:
        rospy.logwarn("No path returned by Dijkstra algo")
        path = []
    else:
        rospy.loginfo("Path recieved!")

    response = pp_msgResponse()
    response.plan = path
    return response


def clean_shutdown():
    cmd_vel.publish(Twist())
    rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node("dijkstra_path_planning_service_server",
                    log_level=rospy.INFO, anonymous=False)
    make_plan_service = rospy.Service(
        "/move_base/make_plan", pp_msg, make_plan)
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rospy.on_shutdown(clean_shutdown)

    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
    rospy.Timer(rospy.Duration(2), rospy.signal_shutdown(
        "Shutting Down"), oneshot=True)
