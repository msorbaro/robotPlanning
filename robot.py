#Morgan Sorbaro
#Robot File for all Robot info
#11.12.17

#import statements
from cs1lib import *
from math import cos, sin, pi
from shapely.geometry import LineString, Point, MultiLineString
from random import  uniform
from bfs_search import bfs_search


#This is the Robot class that holds all the information the Robot needs
class Robot:

    #Robot takes lendth of the arms, the list of thetas, potential constraints and potentital goal
    def __init__(self, l, t, constraints= None, goal=None):
        self.armCoordinates = self.calculateXY(l, t) # get the arm coordinates in X,Y tuples as a list
        self.constraints = constraints #list of constraints
        self.length = l #length of the arm
        self.thetas = t #list of thetas in the start state
        self.goal = goal #goal state
        self.dict = self.buildMap(500) #graph with n amount of states

    #This gets a list of thetas and returns the XY coordinates of all teh points knowing length and theta
    def calculateXY(self, l, t):
        #going to be returned, xy list
        coordinateList = [(0,0)]

        xtot = 0 #total amount of x direction
        ytot = 0 #total amount of y direction
        thetatot = 0 #total amount of theta

        #for each theta value in the list of thetas
        for theta in t:

            #get the curr X with the formula from class
            currX =  l * cos(theta + thetatot) + xtot
            #get the curr y with the formula from class
            currY = l * sin(theta + thetatot) + ytot

            #add these X and Y values to the coordinate List
            coordinateList.append((currX, currY))

            #update x and y and theta totals to reflect the new point addition
            xtot=currX
            ytot=currY
            thetatot += theta

        #return the list
        return coordinateList

    #This method checks the coordinates for a line with the constraints given
    def checkCollision(self, coordinates):
        #create a line
        line = LineString(coordinates)

        #loop through all constraints
        for con in self.constraints:
            #get hte point for constraints
            p  = Point(con[0], con[1])
            #make sure the point has a radius
            c = p.buffer(con[2]).boundary

            #return if any of these things happen and an intersection occurs
            if c.intersects(line) or c.contains(line) or c.touches(line) or c.overlaps(line) or c.within(line):
                return True
        return False


    #this method builds my map to search later
    def buildMap(self, totalNodes):
        #create a dictionary
        dictionary = {}
        #set nodeNum to 0
        nodeNum = 0

        #while nodeNum < TotalNodes
        while nodeNum < totalNodes:
            #choose random theta configuration
            currState = self.chooseRandomConfig()
            #if that random configuration isnt a collision

            if not self.createCollision(currState):
                #add that vertext to dictionary
                dictionary[currState] = set()
                #increase nodeNum by one
                nodeNum+=1
                #calculate k (15) closest points to this current configuration
                neighboringValues = self.calculateNeighbors(currState, dictionary, 5)

                #loop through those calculated values
                for neighbor in neighboringValues:
                    # if the calculated one is not the same as the current one and connect good
                    if self.notequal(neighbor, currState) and self.connectOkay(neighbor, currState):
                        #add this to graph
                        if currState in dictionary:
                            dictionary[currState].add(neighbor)
                            #make sure to add directed edges to both
                            if neighbor in dictionary:
                                dictionary[neighbor].add(currState)
                            else:
                                dictionary[neighbor] = {currState}
                        #create a new entry for the graph
                        else:
                            dictionary[currState] = {neighbor}
                            #makes sure adding directed edges to both
                            if neighbor in dictionary:
                                dictionary[neighbor].add(currState)
                            else:
                                dictionary[neighbor] = {currState}

        #add the start state at the end
        if tuple(self.thetas) not in dictionary:
            dictionary[tuple(self.thetas)] = set(self.calculateNeighbors(tuple(self.thetas), dictionary, 15))
        #add the goal state at the end
        if self.goal not in dictionary:
            dictionary[tuple(self.goal)] = set(self.calculateNeighbors(tuple(self.goal), dictionary, 15))

        #loop through all the points and make sure that the goal points have undirected edges going both ways
        for points in dictionary[self.goal]:
            if points in dictionary:
                dictionary[points].add(self.goal)

        #return final graph
        return dictionary

    #Sees if neighboring values and curr state are equal
    def notequal(self, neighboringValues, currState):
        #loop at each value in the tuple and if they are equal returns false
        for i in range(0, len(neighboringValues)):
            if neighboringValues[i] == currState[i]:
                return False

        return True #returns true if they are not equal

    #checks to make sure the points connect okay without hitting a wall
    def connectOkay(self, neighbor, currState):
        #get the XY coordinates for the neighbor

        neighborXYlist = self.calculateXY(self.length, neighbor)
        #get the XY cooridnates for the current point
        currXYlist = self.calculateXY(self.length, currState)

        if self.checkCollision(neighborXYlist):
            return False

        #loop through the coordinates
        for i in range (0, len(neighborXYlist)):
            #draw a line between the coordinates
            l = [neighborXYlist[i], currXYlist[i]]
            #use this line to detect a collision. IF collision, return false becuase not good
            if self.checkCollision(l):
                return False

        #otherwise everything is okay and we can return true
        return True


    #calculate all the neighbors around the current state
    def calculateNeighbors(self, currState, dictionary, k):
        neighbors = [] #empty list going to return

        #if the length of the dicetionary is less than k, we can ust fill neighbors with the whole dictionary
        if len(dictionary) < k:
            #key is a tuple of thetas
            for key in dictionary:
                neighbors.append(key)
            return neighbors

        #create dictionary of theta-key and its score
        scoredict = {}

        #loop thourgh all dictionary keyds
        for state in dictionary:
            #if list len < 15, add the dictkey to the list
            if(len(neighbors) < k):
                neighbors.append(state)
                scoredict[state] = self.calcScore(currState, state)
            else:
                #find the current largest value and its index in the list
                largest = 0
                ind = -1

                #loop thorugh all the neighbors
                for index in range(0, len(neighbors)):
                    #loop up their scores in the dictionary
                    if scoredict[neighbors[index]] > largest:
                        #if better scores, reset values and add it to neighbors
                        largest = scoredict[neighbors[index]]
                        ind = index

                #if the new keydic is < this value, replace it
                if self.calcScore(currState, state) < largest:
                    neighbors[ind] = state
                    scoredict[state] = self.calcScore(currState, state)
        #return the neighbors
        return neighbors

    #this calculates the score for the new neighbors
    def calcScore(self, currstate, oldstate):
        total = []
        #loops through the states thetas
        for index in range(0, len(currstate)):
            #calculates the difference between the thetas
            total.append(abs(oldstate[index] - currstate[index]))

        #caulcaute the score for the arm
        largest = 0
        for i in total:
            if i > largest:
                largest = i
        return largest

    #curr state is a list of thetas to make the arm
    def createCollision(self, currState):
        #cretae the list of points for the current state
        pointlist = self.calculateXY(self.length, currState)

        #cehck the collision
        return self.checkCollision(pointlist)

    #chooess a random theta configuration
    def chooseRandomConfig(self):
        #want to do the right amount of thetas
        total = len(self.armCoordinates)-1
        thetas = []
        #loop through
        for i in range(0, total):
            #random vlaue between 0 and 2pi
            thetas.append(uniform(0, 2*pi))
        return tuple(thetas) #return value



#This method draws the constraints as circles
def draw_squares(constraints):
    for c in constraints:
        draw_circle(c[0] +200, 200-c[1], c[2])


#this is the problem class so this works well with the BFS
class RobotProblem:
    #problem class needs a start state, goal state and maze just because thats how it was created
    def __init__(self, robot):
        self.robot = robot
        self.goal = robot.goal
        self.start_state = tuple(robot.thetas)
        self.maze = None

    #get sucessors looks through the dictionary and finds the neighbors of the curr place
    def get_successors(self, curr_set, maze):
        #loops of curr set is in dictionary
        if tuple(curr_set) in self.robot.dict:
            #if so returns it
            return self.robot.dict[tuple(curr_set)]
        else: #otherwise returns none
            return None

    # we already did our calculations to amek sure everything we were adding was a safe state
    def safe_state(self, state, maze):
        return True

    #checks that the current state is the same as the goal
    def goal_test(self, state):
        return state == self.goal

#these are the constraints points (x,y, w, h)
points = [(-45, 50, 7, 7), (50, 30, 7, 7), (-35, -15, 7, 7), (20, -15, 7, 7)]
#This is the creation of the robot object
r = Robot(50, [pi / 2, 3 * pi / 2], points, (pi, pi / 2))

k = 0
#This does the bulk of the drawing
def main():
    global k

    #creates the robot problem with the robot
    problem = RobotProblem(r)

    #uses the search list for a solution
    list = (bfs_search(problem).path)

    #loops through the paths
    for i in range (0, k):

        #loops through the amount of arms
        for index in range (0, len(list[i])):
            #gets coordinates

            xy = r.calculateXY(r.length, list[i])
            x1 = xy[index][0] + 200
            y1 = 200 - xy[index][1]
            x2 = xy[index + 1][0] + 200
            y2 = 200 - xy[index + 1][1]

            set_fill_color(1, 0, 0)

            #draws the lines
            draw_line(x1, y1, x2, y2)
    #also draws constraints
    set_fill_color(0, 1, 0)
    if k < len(list):
        k+=1
    draw_squares(r.constraints)

#runs the graphisc
start_graphics(main, width=400, height=400, framerate=3)
