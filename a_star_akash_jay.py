"""
# Title:    Project-3 : A* Algorithm Implementation
# Course:   ENPM661 - Planning for Autonomous Robots
"""

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


def main():
    """main function
    """
    pass


if __name__ == "__main__":
    main()
