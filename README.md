# Robot Arm
This project is an experiment in applying local searching techniques to perform optimal and suboptimal movement planning. A robotic arm manouvres in a 2-dimensional graph avoiding obstacles on its way to a goal point.
## Getting Started

This program is callable from the command line, and has the following format:
```
python3 robot_arm.py  [--joints 45,5,10] [--alg lbs] [--slow] <arm description file> <goal> <obstacle file>..
```
--joints is an optional argument that specifies the starting angle joints for the arm. --alg is an optional argument that specifies the search algorithm to use. It defaults to local beam search. The available algoritms are random restart ("rar"), simulated annealing ("sia") and local beam search ("lbs'). The --slow flag causes the display to update slower to allow one to view the actions individually. <goal> is the goal location of the end effector given as x,y. The <arm description file> is the file that specifies the joints. They are described in a space delimited value file with the first line specifying the first link, second line specifying the second link and so on.  Each line will have the form:
```
R <length> <lower angle> <upper angle>
```
Where R signifies that it is a rotational link of a specified <length>.  The next two numbers are the angle limits (in degrees) for the joint. The remaining arguments of the command are the obstacle files (one polygon per file). An obstacle is described by a space delimited file where each line is the x and y coordinate of a vertex of the polygon listed in the counter clockwise direction around the polygon.
  
## Search algorithms
Random restart chooses an random position to start in and then uses the default hill climb to attempt to reach the goal. If it does not suceed it will generate a new starting position to run from. Simulated Annealing runs one time, and allows suboptimal moves with some probability that decreases over time according to a schedule function. Simulated Annealing can be good for getting off local maxima, but it will only find a use in particular arrangements. Local Beam Search generates k random states, and then chooses the best k succesors from the start states. It continues until it reaches a solution or has no improvement on any of the successors.

### Prerequisites
This program requires the following libraries: argparse, numpy, collections, math,  random, sys, bisect, operator,
os.path, functools, itertools, shapely, matplotlib.

Most of these imports derive from the AIMA search.py and utils.py files that are used in this program.

#### I assisted Micah Zik and Tyler Cooper on this project. Also some of the descriptions of parameters come from the assignment.
