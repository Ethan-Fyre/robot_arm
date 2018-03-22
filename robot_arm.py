# robot_arm.py
# Ethan Sayles
# March 20, 2018
#
# Purpose: do stuff wit robo arm

import argparse
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


def randomrestart(problem):
    initial = aima.hill_climbing(problem)
    print(problem.value(initial))
    tries = [initial]
    while problem.value(tries[-1]) < -.05:
        start = [[random.randint(-90, 90)] for _ in range(len(problem.arms))]
        problem.initial = start
        newstate = aima.hill_climbing(problem)
        print(problem.value(newstate))
        tries.append(newstate)
    return tries


def test():
    start = [[random.randint(-90, 90)] for _ in range(4)]
    testt = ArmClimb([5, 3, 2, 2], [Polygon([(1, 1), (4, 1), (4, 4), (3, 5)]), Polygon([(8, 8), (10, 8), (10, 10)])], start, (2, 7))
    ttest = randomrestart(testt)
    thetastart = [i[0] for i in ttest[0]]
    xdata, ydata = graph(testt.arms, thetastart)
    plt.show()
    plt.plot(testt.goal[0], testt.goal[1], "-ro")
    for i in testt.polylist:
        x, y = i.exterior.xy
        plt.plot(x, y, 'g-')
    axes = plt.gca()
    axes.set_xlim(-1 * np.sum(testt.arms), np.sum(testt.arms))
    axes.set_ylim(-1 * np.sum(testt.arms), np.sum(testt.arms))
    line, = axes.plot(xdata, ydata, 'b-')
    for k in range(len(ttest)):
        for i in range(len(ttest[k][0])):
            thetas = [j[i] for j in ttest[k]]
            xdata, ydata = graph(testt.arms, thetas)
            line.set_xdata(xdata)
            line.set_ydata(ydata)
            plt.draw()
            plt.pause(0.001)
        plt.pause(.01)
    plt.show()


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

test()


