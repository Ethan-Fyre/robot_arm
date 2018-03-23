# robot_arm.py
# Ethan Sayles
# March 20, 2018
#
# Purpose: do stuff wit robo arm

import argparse
import sys
import numpy as np
from shapely.geometry import LineString, Polygon
import search as aima
import matplotlib.pyplot as plt
import random


class ArmClimb(aima.Problem):
    """Subclass of Problem that pertains to this project"""

    def __init__(self, arms, polylist, initial, goal):
        """Constructor for the class"""
        aima.Problem.__init__(self, initial, goal)
        self.arms = arms
        self.polylist = polylist

    def actions(self, state):
        """ Function to specify the actions given a state"""
        acts = [np.zeros(len(state)) for _ in range(2*len(state))]
        for i in range(len(state)):
            acts[2*i][i] = 5
            acts[2*i + 1][i] = -5
        return acts

    def result(self, state, action):
        """ Function to specify the resulting state given an action and state"""
        new_state = [i[:] for i in state]
        for i in range(len(state)):
            new_state[i].append(new_state[i][-1] + action[i])
        thetalist = [i[-1] * np.pi / 180 for i in new_state]
        x = [0] * (len(new_state) + 1)
        y = [0] * (len(new_state) + 1)
        thetasum = np.cumsum(thetalist)
        for i in range(len(new_state)):
            x[i + 1] = self.arms[i] * np.cos(thetasum[i])
            y[i + 1] = self.arms[i] * np.sin(thetasum[i])
        x = np.cumsum(x)
        y = np.cumsum(y)
        xypairs = [[] for _ in range(len(x) - 1)]
        for i in range(len(x) - 1):
            xypairs[i] = [(x[i], y[i]), (x[i+1], y[i+1])]
        lines = [LineString(i) for i in xypairs]
        for i in lines:
            for j in self.polylist:
                if j.intersects(i):
                    return state
        return new_state

    def value(self, state):
        thetalist = [i[-1] * np.pi / 180 for i in state]
        x = [0] * (len(state) + 1)
        y = [0] * (len(state) + 1)
        thetasum = np.cumsum(thetalist)
        for i in range(len(state)):
            x[i + 1] = self.arms[i] * np.cos(thetasum[i])
            y[i + 1] = self.arms[i] * np.sin(thetasum[i])
        xee = np.sum(x)
        yee = np.sum(y)
        val = -1 * np.sqrt((xee - self.goal[0]) ** 2 + (yee - self.goal[1]) ** 2)
        return val

    def sched(self, k=20, lam=.01, limit=500):
        """One possible schedule function for simulated annealing"""
        return lambda t: (k * np.exp(-lam * t) if t < limit else 0)


def graph(l, thetas):
    thetas = [i * np.pi / 180 for i in thetas]
    x = [0] * (len(thetas) + 1)
    y = [0] * (len(thetas) + 1)
    thetasum = np.cumsum(thetas)
    for i in range(len(thetas)):
        x[i+1] = l[i] * np.cos(thetasum[i])
        y[i+1] = l[i] * np.sin(thetasum[i])
    x = np.cumsum(x)
    y = np.cumsum(y)
    return x, y


def local_beam_search(problem, k=5, tol=-.01):
    """From the initial node, keep choosing the neighbor with highest value,
    stopping when no neighbor is better. [Figure 4.2]"""
    current = [aima.Node([[random.randint(-90, 90)] for _ in range(len(problem.arms))]) for _ in range(k)]
    inwall = [True] * len(current)
    while any(inwall):
        for i in current:
            if all([j == i for j in i.expand(problem)]):
                current.append(aima.Node([[random.randint(-90, 90)] for _ in range(len(problem.arms))]))
                current.remove(i)
            else:
                inwall[current.index(i)] = False
    while True:
        neighbors = []
        for i in current:
            neighbors.extend(i.expand(problem))
        if not neighbors:
            break
        neighbor = []
        for _ in range(k):
            newneighbor = aima.argmax_random_tie(neighbors,
                                     key=lambda node: problem.value(node.state))
            neighbor.append(newneighbor)
            neighbors.remove(neighbor[-1])
        for i in neighbor:
            if problem.value(i.state) > tol:
                return i.state
            closer = []
            for j in current:
                closer.append(problem.value(j.state) >= problem.value(i.state))
        if all(closer):
            break
        current = neighbor
    return aima.argmax_random_tie(current,
                                     key=lambda node: problem.value(node.state)).state


def main(lens ,limits, goal, joints, obstacles, alg):
    """Default function to be called when the program executes."""
    robot_arm = ArmClimb(lens, obstacles, joints, goal)
    if alg == "lbs":
        newstate = local_beam_search(robot_arm)
    elif alg == "sta":
        newstate = aima.simulated_annealing(robot_arm, robot_arm.sched())
    elif alg == "rar":
        newstate = aima.hill_climbing(robot_arm)
    else:
        print("algorithm is not implemented. Using local beam search instead")
        alg = "lbs"
        newstate = local_beam_search(robot_arm)
    thetastart = [i[0] for i in newstate]
    xdata, ydata = graph(robot_arm.arms, thetastart)
    plt.show()
    plt.plot(robot_arm.goal[0], robot_arm.goal[1], "-ro")
    for i in robot_arm.polylist:
        x, y = i.exterior.xy
        plt.plot(x, y, 'g-')
    axes = plt.gca()
    axes.set_xlim(-1 * np.sum(robot_arm.arms), np.sum(robot_arm.arms))
    axes.set_ylim(-1 * np.sum(robot_arm.arms), np.sum(robot_arm.arms))
    line, = axes.plot(xdata, ydata, 'b-')
    while True:
        for i in range(len(newstate[0])):
            thetas = [j[i] for j in newstate]
            xdata, ydata = graph(robot_arm.arms, thetas)
            line.set_xdata(xdata)
            line.set_ydata(ydata)
            plt.draw()
            plt.pause(0.0001)
        if alg == "rar":
            if robot_arm.value(newstate) > -.05:
                break
            start = [[random.randint(-90, 90)] for _ in range(len(robot_arm.arms))]
            robot_arm.initial = start
            newstate = aima.hill_climbing(robot_arm)
            plt.pause(.001)
        else:
            break
    print("Final position angles are ",[i[-1] for i in newstate])
    print("Value is ", robot_arm.value(newstate))
    plt.show()


if __name__ == '__main__':

    # Arguments for argparse.
    parser = argparse.ArgumentParser(description='')
    argv = sys.argv[1:]
    parser.add_argument("arm_file", help="")
    parser.add_argument("goal", help="")
    if "--joints" in argv:
        i = argv.index("--joints")
        argv2 = argv[:i] + argv[i+2:]
        if "--alg" in argv2:
            j = argv2.index("--alg")
            argv3 = argv2[:j] + argv2[j + 2:]
            parser.add_argument("ob_files", nargs=argparse.REMAINDER)
            args, joints, alg = parser.parse_args(argv3), argv[i + 1], argv2[j + 1]
        else:
            parser.add_argument("ob_files", nargs=argparse.REMAINDER)
            args, joints, alg = parser.parse_args(argv2), argv[i + 1], "lbs"
    else:
        if "--alg" in argv:
            j = argv.index("--alg")
            argv2 = argv[:j] + argv[j + 2:]
            parser.add_argument("ob_files", nargs=argparse.REMAINDER)
            args, joints, alg = parser.parse_args(argv2), [], argv[j + 1]
        else:
            parser.add_argument("ob_files", nargs=argparse.REMAINDER)
            args, joints, alg = parser.parse_args(argv), [], "lbs"
    lens = []
    limits = []
    obstacles = []
    for i in args.ob_files:
        with open(i) as file:
            coords = []
            for data in file:
                coords.append(tuple([int(i) for i in data.split(' ')]))
        obstacles.append(Polygon(coords))
    with open(args.arm_file) as file:  # Use file to refer to the file object
        for data in file:
            datalist = [i for i in data.split(' ')]
            lens.append(int(datalist[1]))
            limits.append((int(datalist[2]), int(datalist[3])))
    goal = [int(i) for i in args.goal.split(',')]
    if not joints:
        joints = [[random.randint(-90, 90)] for _ in range(len(lens))]
    else:
        joints = [[int(i)] for i in joints.split(',')]
main(lens, limits, goal, joints, obstacles, alg)


