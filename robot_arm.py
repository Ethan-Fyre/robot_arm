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

    def __init__(self, arms, polylist, initial, limits, goal):
        """Constructor for the class"""
        aima.Problem.__init__(self, initial, goal)
        self.arms = arms
        self.polylist = polylist
        self.limits = limits

    def random_start(self, state):
        """Function ensures the arm will not start inside an obstacle"""
        thetalist = [i[-1] * np.pi / 180 for i in state]
        x = [0] * (len(state) + 1)
        y = [0] * (len(state) + 1)
        thetasum = np.cumsum(thetalist)
        for i in range(len(state)):
            x[i + 1] = self.arms[i] * np.cos(thetasum[i])
            y[i + 1] = self.arms[i] * np.sin(thetasum[i])
        x = np.cumsum(x)
        y = np.cumsum(y)
        xypairs = [[] for _ in range(len(x) - 1)]
        for i in range(len(x) - 1):
            xypairs[i] = [(x[i], y[i]), (x[i + 1], y[i + 1])]
        lines = [LineString(i) for i in xypairs]
        for i in lines:
            for j in self.polylist:
                if j.intersects(i):
                    new = [[random.randint(limits[i][0], limits[i][1])] for i in range(len(state))]
                    return self.random_start(new)
        return state

    def actions(self, state):
        """ Function to specify the actions given a state"""
        acts = [np.zeros(len(state)) for _ in range(2*len(state))]
        x = -1 * self.value(state)
        if x < 3:
            for i in range(len(state)):
                acts[2 * i][i] = x * 5
                acts[2 * i + 1][i] = -x * 5
        else:
            for i in range(len(state)):
                acts[2*i][i] = 10
                acts[2*i + 1][i] = -10
        for i in acts:
            for j in range(len(i)):
                if not self.limits[j][0] <= state[j][-1] <= self.limits[j][1]:
                    acts.remove(i)
                    break
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
        """Function to calculate the value to be maximized by the local search algorithms"""
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

    def sched(self, k=1, lam=.01, limit=100):
        """One possible schedule function for simulated annealing"""
        return lambda t: (k * np.exp(-lam * t) if t < limit else 0)


def graph(l, thetas):
    """Function to return a list of xs and ys given lengths and angles"""
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


def hill_climbing_with_tol(problem, tol=.01):
    """From the initial node, keep choosing the neighbor with highest value,
    stopping when no neighbor is better. (Slightly modified version of the
    aima hill_climbing function)"""
    current = aima.Node(problem.random_start(problem.initial))
    while True:
        neighbors = current.expand(problem)
        if not neighbors:
            break
        neighbor = aima.argmax_random_tie(neighbors,
                                     key=lambda node: problem.value(node.state))
        if problem.value(neighbor.state) <= problem.value(current.state) or problem.value(current.state) > -1*tol:
            break
        current = neighbor
    return current.state


def simulated_annealing_with_tol(problem, tol=.01):
    """Modified version of the aimi simulated annealing function that takes
    a problem and a tolerance"""
    schedule = problem.sched()
    current = aima.Node(problem.random_start(problem.initial))
    for t in range(sys.maxsize):
        T = schedule(t)
        if T == 0 or problem.value(current.state) > -1 * tol:
            return current.state
        neighbors = current.expand(problem)
        if not neighbors:
            return current.state
        next = random.choice(neighbors)
        delta_e = problem.value(next.state) - problem.value(current.state)
        if delta_e > 0 or aima.probability(np.exp(delta_e / T)):
            current = next


def local_beam_search(problem, k=5, tol=.01):
    """From the initial k nodes, keep choosing the k neighbors with highest value,
    stopping when no neighbors are better (or it is withing the tolerance)."""
    current = [aima.Node(problem.random_start([[random.randint(problem.limits[i][0], problem.limits[i][1])] for i in range(len(problem.arms))])) for _ in range(k)]
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
        closer = []
        for i in neighbor:
            if problem.value(i.state) > -1 * tol:
                return i.state
            for j in current:
                closer.append(problem.value(j.state) >= problem.value(i.state))

        if all(closer):
            break
        current = neighbor
    return aima.argmax_random_tie(current,
                                     key=lambda node: problem.value(node.state)).state


def main(lens, limits, goal, joints, obstacles, alg, slow):
    """Default function to be called when the program executes."""
    print(limits)
    robot_arm = ArmClimb(lens, obstacles, joints, limits, goal)
    if alg == "lbs":
        print("Now beginning Local Beam Search:")
        newstate = local_beam_search(robot_arm)
    elif alg == "sia":
        print("Now beginning Simulated Annealing:")
        newstate = simulated_annealing_with_tol(robot_arm)
    elif alg == "rar":
        print("Now beginning Random Restart:")
        newstate = hill_climbing_with_tol(robot_arm)
    else:
        print("Algorithm is not implemented. Beginning Local Beam Search instead:")
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
    elbows, = axes.plot(xdata, ydata, 'b.')
    while True:
        for i in range(len(newstate[0])):
            thetas = [j[i] for j in newstate]
            xdata, ydata = graph(robot_arm.arms, thetas)
            line.set_xdata(xdata)
            line.set_ydata(ydata)
            elbows.set_xdata(xdata)
            elbows.set_ydata(ydata)
            plt.draw()
            if slow:
                plt.pause(.1)
            else:
                plt.pause(0.0001)
        if alg == "rar":
            if robot_arm.value(newstate) > -.01:
                break
            start = [[random.randint(limits[i][0], limits[i][1])] for i in range(len(robot_arm.arms))]
            robot_arm.initial = start
            newstate = hill_climbing_with_tol(robot_arm)
            plt.pause(.001)
        else:
            break
    print("Final position angles are ", [i[-1] for i in newstate])
    print("Distance is ", -1 * robot_arm.value(newstate))
    plt.show()


if __name__ == '__main__':
    # Arguments for argparse.
    parser = argparse.ArgumentParser(description='')
    parser.add_argument("arm_file", help="")
    parser.add_argument("goal", help="")
    parser.add_argument("--slow", action='store_true')
    parser.add_argument("--joints", default=[])
    parser.add_argument("--alg", default="lbs")
    parser.add_argument("ob_files", nargs="+")
    args = parser.parse_args(sys.argv[1:])

    lens = []
    limits = []
    obstacles = []
    # read in obstacle data
    for i in args.ob_files:
        with open(i) as file:
            coords = []
            for data in file:
                coords.append(tuple([int(i) for i in data.split(' ')]))
        obstacles.append(Polygon(coords))

    # read in arm data
    with open(args.arm_file) as file:
        for data in file:
            datalist = [i for i in data.split(' ')]
            lens.append(int(datalist[1]))
            limits.append((int(datalist[2]), int(datalist[3])))
    goal = [int(i) for i in args.goal.split(',')]
    if not args.joints:
        joints = [[random.randint(limits[i][0], limits[i][1])] for i in range(len(lens))]
    else:
        joints = [[int(i)] for i in args.joints.split(',')]
main(lens, limits, goal, joints, obstacles, args.alg, args.slow)


