# Path Planning
This project is an experiment in applying local searching techniques to perform optimal and suboptimal movement planning. rid where each grid element is a unit square containingspeed of 0. The path allows 8-directional movement, where each step is from the center of one square to the center of the next square.
## Getting Started

This program is callable from the command line, and has the following format:
```
python3 project2.py <grid filename> <start coordinates> <stop coordinates>
```
There is an optional argument '--alg' which defaults to 'a_star', but can be set to 'ucs' for uniform cost search, and 'bfs' for breadth first search. Entering in any other value will just use A* search. Grid is a csv file, and start, stop are of the form r,c specifying the row and column.

## Properties of search algorithms

The first search is the breadth-first search (BFS). The BFS is complete for this problem, as the branching factor is finite (3 for corners, 5 for edges, 8 for interior pieces) and the goal will be reachable eventually. BFS is only optional in the case that all the grid locations have the same speed; otherwise it is not guarenteed to be optimal. The time to find a solution is O(b^d), where b is the branching factor, and d is the depth of the solution. The space complexity is a factor of the time complexity, and is O(b^d) as well. The second search is the uniform-cost search (UCS). The UCS is optimal and complete for the grid problem, as it explores the nodes with the lowest path cost. However, the worst-case time and space complexity for the UCS is O(b^(1+floor(C* / e))) where C* is the cost of the optimal solution, and every action costs at least e. The final search is the A* which is guarenteed to be optimal for an admissable heuristic function. Given that this problem is in finding a path, the straight line distance was uesd as the hueristic function.The drawback to A* lies in the exponential complexity. The complexity is defined as O(b^(h* - h)) where h* is the actual cost of getting from the inital state to the goal state.

## Calculation of step cost
The derivation of the step costs lies in geometry and unit analysis. The squares are unit squares, and all actions move from the center of one square to the center of the next square. As such the time spent in one square while moving horizontally/vertically is 1/2 distance. The speed at this time is the value in the grid location, and is distance/time. In order to get the time to leave one square in a horizontal/vertical motion we must multiply the distance by the inverse of the speed. This results in the equation 1/2 * 1/speed1, where speed1 is the speed of the start location. By the same reasoning the time through  the second square is 1/2 * 1/speed2. Therefore, the total time to transition from the center of one square to the center of another is 1/2 * (1/speedq + 1/speed2) through factoring. However, this equation does not account for diagonal movement. The diagonal distance to leave a square can be found through geometry to be 1/sqrt(2). Using the reasoning as the previous section, the time to move diagonally is 1/sqrt(2) * (1/speed1 + 1/speed2).
### Prerequisites
This program requires the following libraries: argparse, numpy, collections, math,  random, sys, bisect, operator,
os.path, functools, itertools.

Most of these imports derive from the AIMA search.py and utils.py files that are used in this program.

