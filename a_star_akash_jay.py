"""
# Title:    Project-3 : A* Algorithm Implementation
# Course:   ENPM661 - Planning for Autonomous Robots
"""

from numpy import cos, sin, deg2rad, round, sqrt

import time
import heapq
import cv2
import numpy as np

__author__ = "Jay Prajapati, Akash Parmar"
__copyright__ = "Copyright 2023, Project-3"
__credits__ = ["Jay Prajapati", "Akash Parmar"]
__license__ = "MIT"
__version__ = "1.0.1"
__email__ = "jayp@umd.edu", "akasparm@umd.edu"


class NODE:
    """class for data structure to store the node information
    """

    def __init__(self, coordinates, cost, preced=None, cost_goal=0):
        """Constructor

        Args:
            coordinates (list): 
                [x, y, theta] for the node
            cost (float): 
                cost of the node
            preced (<class 'NODE'>, optional): 
                parent node for the node. Defaults to None.
            cost_goal (int, optional): 
                Cost to reach the goal for the current node. Defaults to 0.
        """
        self.coordinates = coordinates
        self.x_cord = coordinates[0]
        self.y_cord = coordinates[1]
        self.theta = coordinates[2]
        self.cost = cost
        self.preced = preced
        self.cost_goal = cost_goal

    def __lt__(self, other):
        """Defining the comparision operator
        """
        comparision = self.cost + self.cost_goal < other.cost + other.cost_goal
        return comparision


def createCanvas():
    """Creates the obstacles

    Returns:
        np.ndarray: Binary image containing the obstacle data
    """
    # Create an empty frame
    map = np.zeros((canvas_height, canvas_width), dtype=np.uint8)

    # set the offset
    offset = clearance + robot_radius

    # Treversing through each pixel
    for i in range(canvas_height):
        for j in range(canvas_width):

            # defining the rectangles
            if (j >= 100-(offset) and j <= 150+(offset) and i >= 0 and i <= 100+(offset)):
                map[i][j] = 1

            if (j >= 100-(offset) and j <= 150+(offset) and i >= 150-(offset) and i < 250):
                map[i][j] = 1

            # defining the polygon
            if (j >= 300-(offset)-75*np.cos(np.deg2rad(30)) and j <= 300+(offset)+75*np.cos(np.deg2rad(30))):
                if (37.5*j + 64.95*i > 14498-649.5) and (-37.5*j + 64.95*i < 1742+649.5):
                    if (37.5*j + 64.95*i < 24240+649.5) and (37.5*j - 64.95*i < 8002.5+649.5):
                        map[i][j] = 1

            # defining the traingle
            if (j >= 460-(offset)):
                if (i >= 20 and i <= 230):
                    if (-2*j + i > -895-20) and (2*j + i < 1145+20):
                        map[i][j] = 1

            # defining the borders
            if (j >= 0 and j < (offset)) or (i >= 0 and i < (offset)) or \
                    (i >= 250-(offset) and i < 250) or (j >= 600-(offset) and j < 600):
                map[i][j] = 1
    return map


def checkSolvable(x, y, map):
    """checks whether the given coordinates are on the obstacles or not

    Args:
        x (int): x-coordinate
        y (int): y-coordinate
        map (np.ndarray): canvas

    Returns:
        bool: Flag for the validity of the point
    """
    if (map[y][x] == 1).all():
        return False
    return True


def animationCanvas():
    """Creates the animation canvas

    Returns:
        np.ndarray: Animation canvas image
    """

    # Create an empty frame
    image = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)

    # set the offset
    offset = clearance + robot_radius

    # colors for the objects
    clearance_color = (34, 34, 43)
    radius_color = (255, 255, 255)
    object_color = (0, 117, 213)

    # Treversing through each pixel
    for i in range(image.shape[0]):
        for j in range(image.shape[1]):

            # Representating robot radius offset

            if (j >= 100-(offset) and j <= 150+(offset) and i >= 0 and i <= 100+(offset)):
                image[i][j] = radius_color

            if (j >= 100-(offset) and j <= 150+(offset) and i >= 150-(offset) and i < 250):
                image[i][j] = radius_color

            if (j >= 300-(offset)-75*np.cos(np.deg2rad(30)) and j <= 300+(offset)+75*np.cos(np.deg2rad(30))):
                if (37.5*j + 64.95*i > 14498-649.5) and (-37.5*j + 64.95*i < 1742+649.5):
                    if (37.5*j + 64.95*i < 24240+649.5) and (37.5*j - 64.95*i < 8002.5+649.5):
                        image[i][j] = radius_color

            if (j >= 460-(offset)):
                if (i >= 20 and i <= 230):
                    if (-2*j + i > -895-20) and (2*j + i < 1145+20):
                        image[i][j] = radius_color

            # Representating Clearance

            if (j >= 100-5 and j <= 150+5 and i >= 0 and i <= 100+5):
                image[i][j] = clearance_color

            if (j >= 100-5 and j <= 150+5 and i >= 150-5 and i < 250):
                image[i][j] = clearance_color

            if (j >= 300-robot_radius-75*np.cos(np.deg2rad(30)) and j <= 300+robot_radius+75*np.cos(np.deg2rad(30))):
                if (37.5*j + 64.95*i > 14498-649.5/2) and (-37.5*j + 64.95*i < 1742+649.5/2):
                    if (37.5*j + 64.95*i < 24240+649.5/2) and (37.5*j - 64.95*i < 8002.5+649.5/2):
                        image[i][j] = clearance_color

            if (j >= 460-robot_radius):
                if (i >= 25 and i <= 225):
                    if (-2*j + i > -895-10) and (2*j + i < 1145+10):
                        image[i][j] = clearance_color

            # Representating the Objects

            if (j >= 100 and j <= 150 and i >= 0 and i <= 100):
                image[i][j] = object_color

            if (j >= 100 and j <= 150 and i >= 150 and i < 250):
                image[i][j] = object_color

            if (j >= 300-75*np.cos(np.deg2rad(30)) and j <= 300+75*np.cos(np.deg2rad(30))):
                if (37.5*j + 64.95*i > 14498) and (-37.5*j + 64.95*i < 1742):
                    if (37.5*j + 64.95*i < 24240) and (37.5*j - 64.95*i < 8002.5):
                        image[i][j] = object_color

            if (j >= 460):
                if (-2*j + i > -895) and (2*j + i < 1145):
                    image[i][j] = object_color

            # Representating the Frame border

            if (j >= 0 and j < (offset)) or (i >= 0 and i < (offset)) or \
                    (i >= 250-(offset) and i < 250) or (j >= 600-(offset) and j < 600):
                image[i][j] = radius_color

            if (j >= 0 and j < (robot_radius)) or (i >= 0 and i < (robot_radius)) or \
                    (i >= 250-(robot_radius) and i < 250) or (j >= 600-(robot_radius) and j < 600):
                image[i][j] = clearance_color
    return image


def roundVals(number):
    """Rounds the given number to nearest 0.5

    Args:
        number (float): Number to be rounded

    Returns:
        float: Rounded value
    """
    return np.around(number*2.0)/2.0


def movePositive60(coordinates, cost):
    """To move the robot forward 
       at an angle of positive 60 degrees

    Args:
        coordinates (list): [x, y, theta]
        cost (float): total cost of the node

    Returns:
        lsit: [[x, y, theta], cost] updated node info
    """
    x, y, theta = coordinates
    theta += 60
    x = (x + (step_size * cos(deg2rad(theta))))
    y = (y + (step_size * sin(deg2rad(theta))))
    cost += 1
    return [[x, y, theta], cost]


def movePositive30(coordinates, cost):
    """To move the robot forward 
       at an angle of positive 30 degrees

    Args:
        coordinates (list): [x, y, theta]
        cost (float): total cost of the node

    Returns:
        lsit: [[x, y, theta], cost] updated node info
    """
    x, y, theta = coordinates
    theta += 30
    x = (x + (step_size * cos(deg2rad(theta))))
    y = (y + (step_size * sin(deg2rad(theta))))
    cost += 1
    return [[x, y, theta], cost]


def moveForward(coordinates, cost):
    """To move the robot forward

    Args:
        coordinates (list): [x, y, theta]
        cost (float): total cost of the node

    Returns:
        lsit: [[x, y, theta], cost] updated node info
    """
    x, y, theta = coordinates
    x = (x + (step_size * cos(deg2rad(theta))))
    y = (y + (step_size * sin(deg2rad(theta))))
    cost += 1

    return [[x, y, theta], cost]


def moveNegative30(coordinates, cost):
    """To move the robot forward 
       at an angle of negative 30 degrees

    Args:
        coordinates (list): [x, y, theta]
        cost (float): total cost of the node

    Returns:
        lsit: [[x, y, theta], cost] updated node info
    """
    x, y, theta = coordinates
    theta -= 30
    x = (x + (step_size * cos(deg2rad(theta))))
    y = (y + (step_size * sin(deg2rad(theta))))
    cost += 1
    return [[x, y, theta], cost]


def moveNegative60(coordinates, cost):
    """To move the robot forward 
       at an angle of negative 60 degrees

    Args:
        coordinates (list): [x, y, theta]
        cost (float): total cost of the node

    Returns:
        lsit: [[x, y, theta], cost] updated node info
    """
    x, y, theta = coordinates
    theta -= 60
    x = (x + (step_size * cos(deg2rad(theta))))
    y = (y + (step_size * sin(deg2rad(theta))))
    cost += 1
    return [[x, y, theta], cost]


def childValidity(child):
    """Checks whether the child is valid or not

    Args:
        child (list): [[x, y, theta], cost] child node info

    Returns:
        bool: Validity of child node
    """
    if child[0][0] >= 0 and child[0][0] <= canvas.shape[1]-1 and child[0][1] >= 0 and child[0][1] <= canvas.shape[0]-1:
        if canvas[int(round(child[0][1]))][int(round(child[0][0]))] == 0:
            return True
    return False


def generateChildren(node):
    """Generates all the valid children nodes of a given node

    Args:
        node (<class 'NODE'>): Given node

    Returns:
        List: lsit containing all the children nodes
    """
    # Get the x, y and theta of the node
    x_cord, y_cord, theta = node.coordinates

    # Get the cost of the node
    cost = node.cost

    # Creating list for the succeeding nodes
    succ_nodes = []

    # Generating and appending the succeeding
    # nodes using all the movement functions

    p60 = movePositive60([x_cord, y_cord, theta], cost)
    # check validity
    if childValidity(p60):
        succ_nodes.append([
            [roundVals(p60[0][0]), roundVals(p60[0][1]), p60[0][2]],
            p60[1]
        ])

    p30 = movePositive30([x_cord, y_cord, theta], cost)
    # check validity
    if childValidity(p30):
        succ_nodes.append([
            [roundVals(p30[0][0]), roundVals(p30[0][1]), p30[0][2]],
            p30[1]
        ])

    frd = moveForward([x_cord, y_cord, theta], cost)
    # check validity
    if childValidity(frd):
        succ_nodes.append([
            [roundVals(frd[0][0]), roundVals(frd[0][1]), frd[0][2]],
            frd[1]
        ])

    n30 = moveNegative30([x_cord, y_cord, theta], cost)
    # check validity
    if childValidity(n30):
        succ_nodes.append([
            [roundVals(n30[0][0]), roundVals(n30[0][1]), n30[0][2]],
            n30[1]
        ])

    n60 = moveNegative60([x_cord, y_cord, theta], cost)
    # check validity
    if childValidity(n60):
        succ_nodes.append([
            [roundVals(n60[0][0]), roundVals(n60[0][1]), n60[0][2]],
            n60[1]
        ])

    return succ_nodes


def checkVisitedRegion(x, y, theta):
    """check duplicate nodes

    Args:
        x (float): x-coordinate
        y (float): y-coordinate
        theta (int): orientation of the robot at node

    Returns:
        bool: Node duplicacy flag
    """
    # Update the value of x with the given threshold
    xn = int(roundVals(x)/visited_threshold)

    # Update the value of y with the given threshold
    yn = int(roundVals(y)/visited_threshold)

    # Update the value of theta with the given threshold
    # and handle the negative values of theta
    if theta <= 0:
        thetan = int(360/(360-theta))-1
    else:
        thetan = int(360/theta)-1

    # Check duplicacy
    if visited_mat[yn][xn][thetan] == 0:
        visited_mat[yn][xn][thetan] = 1
        return True

    return False


def aStar(start_node, goal_node):
    """Node Exploration using a* algorithm

    Args:
        start_node (<class 'NODE'>): Start point node
        goal_node (<class 'NODE'>): Goal point node

    Returns:
        dict, np.ndarray, list: node_graph, animation_canvas, animation_frames
    """
    # Create a canvas for animation
    animation_canvas = animationCanvas()

    # Create a list to save all the naimation frames
    animation_frames = []

    # Create a dictionary to store the node graph
    node_graph = {}

    # Create dictionaries for the open_list and closed_list
    open_list = {}
    closed_list = {}

    # Create a queue
    queue = []

    # Add the start node in the openlist
    open_list[str([start_node.x_cord, start_node.y_cord])] = start_node

    # Add the initial node to the heap
    heapq.heappush(queue, [start_node.cost, start_node])

    # initialize the while loop
    i = 0
    while len(queue) != 0:
        # get the element from the heap
        fetched_ele = heapq.heappop(queue)

        # fetch the node from that element
        current_node = fetched_ele[1]

        # add the node to the node graph dictionary
        node_graph[str([current_node.x_cord, current_node.y_cord])
                   ] = current_node

        # mark the node on the canvas
        cv2.circle(
            img=animation_canvas,
            center=(int(current_node.x_cord), int(current_node.y_cord)),
            radius=1,
            color=(0, 0, 255),
            thickness=-1
        )

        # check if reached to goal node
        if sqrt((current_node.x_cord-goal_node.x_cord) ** 2 + (current_node.y_cord-goal_node.y_cord)**2) < goal_threshold:
            # assign the parent
            goal_node.preced = current_node.preced

            # assign the cost
            goal_node.cost = current_node.cost

            # print message
            print("#  ")
            print("#  Found the goal node")

            # break the loop
            break

        # check if the node is in closed list
        if str([current_node.x_cord, current_node.y_cord]) in closed_list:
            continue
        # else add it to the closed list
        else:
            closed_list[str(
                [current_node.x_cord, current_node.y_cord])] = current_node

        # delete the element from the open list
        del open_list[str([current_node.x_cord, current_node.y_cord])]

        # Generate the children nodes
        child_list = generateChildren(current_node)

        # Process each child node
        for child in child_list:
            # Get the x, y, theta
            child_x, child_y, child_theta = child[0]

            # get the cost
            child_cost = child[1]

            # mark the child node
            cv2.circle(
                img=animation_canvas,
                center=(int(child_x), int(child_y)),
                radius=1,
                color=(0, 255, 0),
                thickness=-1
            )

            # Save the animation frame at a rate of 1000 frames
            if i % 1000 == 0:
                animation_frames.append(animation_canvas.copy())

            # skip if the child is already in the closed list
            if str([child_x, child_y]) in closed_list:
                continue

            # calculate the cost to goal for the child
            child_cost_goal = sqrt((goal_node.x_cord-child_x)**2 +
                                   (goal_node.y_cord-child_y)**2)

            # generate a node for the child
            child_node = NODE(
                coordinates=[child_x, child_y, child_theta],
                cost=child_cost,
                preced=current_node,
                cost_goal=child_cost_goal
            )

            # check duplicate nodes
            if checkVisitedRegion(child_x, child_y, child_theta):
                # check if the node is in openlist
                if str([child_x, child_y]) in open_list:
                    # if yes compare the cost of the node
                    # if it is smaller then update the cost
                    if child_node.cost < open_list[str([child_x, child_y])].cost:
                        open_list[str([child_x, child_y])].cost = child_cost
                        open_list[str([child_x, child_y])
                                  ].preced = current_node
                # else add the node in the open list
                else:
                    open_list[str([child_x, child_y])] = child_node

                # add the node to the heap
                heapq.heappush(
                    queue, [(child_cost+child_cost_goal), child_node])
        # Increase the operator
        i += 1

    return node_graph, animation_canvas, animation_frames


def backTrack(node_graph, goal_node):
    """Track the path to the goal

    Args:
        node_graph (dict)
        goal_node (<class 'NODE'>): Goal point node

    Returns:
        list: path from the goal to the start node
    """
    # create list to save the path
    path = []

    # add the goal node coordinates
    path.append([int(goal_node.x_cord), int(goal_node.y_cord)])

    # update the parent to the goal node
    parent = list(node_graph.items())[-1][1].preced

    # Continue tracking the parent until reached the start node
    while parent is not None:
        # append the coordinates
        path.append([int(parent.x_cord), int(parent.y_cord)])
        # update the parent
        parent = parent.preced
    return path


def saveAnimation(animation_array):
    """Generates an Animation and Saves the video

    Args:
        animation_array (_type_): _description_
    """
    print("\n")
    print("#"*80)
    print("Generating the video file.")
    # create the video object
    video = cv2.VideoWriter(
        'shortest.mp4', cv2.VideoWriter_fourcc(*'MP4V'), 50, (600, 250))

    # write all the saved frames in the video
    for i in range(len(animation_array)):
        frame = cv2.flip(animation_array[i], 0)
        video.write(frame)
        cv2.imshow("Exploration", frame)
        cv2.waitKey(1)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    video.release()

    print("Video file generated succesfully.")
    print("#"*80)
    print("\n")


def main():
    """main function
    """
    # declare the goabl variables
    global canvas, canvas_height, canvas_width
    global step_size, clearance, robot_radius
    global goal_threshold, visited_mat, visited_threshold

    # declare the canvas height and canvas width
    canvas_height, canvas_width = 250, 600

    # threshold to check the duplicate nodes
    visited_threshold = 0.5

    # matrix to store the info of visited region
    visited_mat = np.zeros((500, 1200, 12), dtype=np.uint8)

    # threshold for the goal periphery
    goal_threshold = 1.5

    # Print title on the terminal
    print("#"*80)
    print("#  Reach the goal (Path Planning using A*)")
    print("#  Project-3")
    print("#  ")
    print("#  by")
    print("#  Akash Parmar")
    print("#  Jay Prajapati")
    print("#"*80)

    # initialize a loop to get input from valid points
    loop = True
    while loop:
        print("\nEnter your choice for mode of operation,")
        print("\nType 1 for selecting the parameters manually")
        print("Type 2 for preset parameters")

        # get the choice from the user for selecting the points manually or default
        choice = int(input("\nYour Choice: "))

        # get input for manual extry
        if choice == 1:
            start_x = int(input("\nEnter x coordinate of Start Point: "))
            start_y = int(input("Enter y coordinate of Start Point: "))
            start_theta = int(input("Enter theta coordinate of Start Point: "))
            if start_theta % 30 != 0:
                print("\n")
                print("#"*80)
                print("#  ERROR")
                print("#  Theta should be a multiple of 30")
                print("#"*80)
                continue

            goal_x = int(input("Enter x coordinate of Goal Point: "))
            goal_y = int(input("Enter y coordinate of Goal Point: "))
            goal_theta = int(input("Enter theta coordinate of Goal Point: "))
            if start_theta % 30 != 0:
                print("\n")
                print("#"*80)
                print("#  ERROR")
                print("#  Theta should be a multiple of 30")
                print("#"*80)
                continue

            clearance = int(input("Enter the clearance: "))
            if clearance != 5:
                print("\n")
                print("#"*80)
                print("#  ERROR")
                print("#  The acceptable clearance is 5 as per the instructions")
                print("#"*80)
                continue

            robot_radius = int(input("Enter the robot radius: "))
            if robot_radius != 5:
                print("\n")
                print("#"*80)
                print("#  ERROR")
                print("#  The acceptable robot radius is 5 as per the instructions")
                print("#"*80)
                continue

            step_size = int(float(input("Enter the step size: ")))
            if step_size not in range(1, 11):
                print("\n")
                print("#"*80)
                print("#  ERROR")
                print("#  The acceptable step size is 1 <= step_size <= 10")
                print("#"*80)
                continue

        # default setup points
        elif choice == 2:
            start_x = 10
            start_y = 10
            start_theta = 60

            goal_x = 500
            goal_y = 200
            goal_theta = 0

            clearance = 5
            robot_radius = 5
            step_size = 1

        # error message for the invalid choices
        else:
            print("Invalid Choice")

        # display the coordinates
        print("\nStart Point = [", start_x, ", ",
              start_y, ", ", start_theta, "]")
        print("Goal Point = [", goal_x, ", ", goal_y, ", ", goal_theta, "]")

        # make the canvas
        canvas = createCanvas()

        # check the solvability
        if checkSolvable(start_x, start_y, canvas):
            if checkSolvable(goal_x, goal_y, canvas):

                # initiale the start and the goal node
                start_node = NODE([start_x, start_y, start_theta], 0)
                goal_node = NODE([goal_x, goal_y, goal_theta], 0)

                print("\n")
                print("#"*80)
                print("#  Finding the goal node")

                # note the start time
                start_time = time.time()

                # Explore nodes
                node_graph, animation_canvas, animation_frames = aStar(
                    start_node, goal_node)

                # generate path
                return_path = backTrack(node_graph, goal_node)

                # note the end time
                end_time = time.time()

                print("#  ")
                print("#  The output was processed in ",
                      end_time-start_time, " seconds.")
                print("#"*80)

                # Mark the path
                for a in return_path[::-1]:
                    cv2.circle(animation_canvas, a, 1, (255, 0, 0), -1)
                    animation_frames.append(animation_canvas.copy())
                break

        else:
            # print the error message
            print("\n")
            print("#"*80)
            print("#  ERROR")
            print("#  The start point or goal point is on the obstacle")
            print("#"*80)

    # Visualize the exploration and save the animation
    saveAnimation(animation_frames)


if __name__ == "__main__":
    main()
