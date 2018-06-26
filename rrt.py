#Morgan Sorbaro
#11.14.17
#CS76

# using shapely for collision detection
from shapely.geometry import Polygon, Point
#other import statements
from random import uniform
from math import pi, sqrt, pow
from planarsim import *
from cs1lib import *
from bfs_search import bfs_search


####CREATING THE RRT METHODS AND VARIABLES

#obsticals for the RRT problem
obsticals = [Polygon([(0,0), (50, 0), (0, 200), (50, 200)]), Polygon([(300, 200), (200, 300), (300, 300)])]
coordinates = [((0,0), (50, 0), (0, 200), (50, 200)), ((200, 200), (300, 200), (200, 300), (300, 300))]

#q init looks like (x, y, theta) k = #, delta_q = #
#This is the method to create the RRT map
def create_RRT(qinit, k, delta_q):
    # create map with a start point
    graph = {}
    graph[tuple(qinit)] = set()

    # from 1 - as many nodes as we want
    for i in range(1, k):
        #get random value
        qrand = rand_config()
        #get the nearest vertex
        qnear = nearest_vertex(qrand, graph)
        #get new Q
        qnewlist = new_conf(qnear, delta_q)

        #loop through all the next potential directions
        for qnew in qnewlist:
            #add q new as a vertex and q near and q new edges
            if qnew in graph:
                graph[qnew].add(qnear)
            else:
                graph[qnew] = set()
                graph[qnew].add(qnear)
            #add both directions
            if qnear in graph:
                graph[qnear].add(qnew)
            else:
                graph[qnear] = set()
                graph[qnear].add(qnew)


    # return dictionary
    return graph

#Returns a random place on the graph
def rand_config():
    return (uniform(0, 500), uniform(0, 500), uniform(0, 2*pi))

#nearest vertex on the graph
def nearest_vertex(qrand, graph):
    # d = infinity
    d = 1000000000
    # vnew = Null
    vnew = None
    #for each vertext in graph
    for v in graph:
        #if the metric p(q,v) < d
        if p(qrand, v) < d:
            # d = p(q,v)
            d = p(qrand, v)
            # vnew = v
            vnew = v

    return vnew

def p(qrand, v):
    xtot = abs(qrand[0]-v[0])
    ytot = abs(qrand[1]- v[1])
    return abs(sqrt((xtot * xtot) + (ytot *ytot)))

#get list of new configurations that the car can go
def new_conf(qnear, delta_q):
    # move from q near 'distance' towards q rand
    returnlist = []

    #branch by each 6 new potential locations in control rs
    for control in controls_rs:
        #apply the control transformation
        start = transform_from_config(qnear)
        resulting_transform = single_action(start, control, delta_q)
        resulting_configuration = config_from_transform(resulting_transform)

        #if the result does not intersect aynthing, add it to the list
        if isgood(resulting_configuration):
            returnlist.append(resulting_configuration)
    #return the list
    return returnlist


#Checks to make sure the configuration is good
def isgood(config):
    print(config)
    #loops through the obsticals and sees if there is an intersection
    for obstical in obsticals:
        if obstical.contains(Point(config[0], config[1])):
            return False #if intersection, return false
    return True #otherwise, return true

#create the resulting RRT map using the start coordinate of 100, 100, pi, 1000 nodes, and a lendth of 10
result = create_RRT([100,100, pi], 1000, 10)
#final location we want to car to end up
final = (400, 300, pi)
#adding the final location to the map (connecting it to the nearest vertex)
qnear = nearest_vertex(final, result)
#add the final location
result[qnear].add(final)


###CREATING THE RRT PROBLEM FOR BFS

#class to hold the RRT problem as needed by how i implemented my BFS in maze world
class RRTProblem():

    #initalize the problem with resulting maze, start, goal and maze
    def __init__(self, start, result, goal):
        self.result = result #large dictionary map
        self.goal = goal
        self.start_state = start
        self.maze = None #needed from maze world bfs

    # get sucessors looks through the dictionary and finds the neighbors of the curr place
    def get_successors(self, curr_set, maze):
        # loops of curr set is in dictionary
        if tuple(curr_set) in self.result:
                # if so returns it
            return self.result[tuple(curr_set)]
        else:  # otherwise returns none
            return None

    # we already did our calculations to amek sure everything we were adding was a safe state
    def safe_state(self, state, maze):
        return True

    # checks that the current state is the same as the goal
    def goal_test(self, state):
        return state == self.goal

#create the RRT problem with the start, result and final states
rrt = RRTProblem((100,100, pi), result, final)
#get the path from the rrt
path = bfs_search(rrt).path


#### DRAWING METHOD TO VISUALIZE ALL THIS
k = 0
def draw():
    global k
    clear()
    #first lets draw the background map -> loop through all the verticies in the map
    for vert in result:
        neighbors = result[vert] #get the neighboir of the current v
        #for each neighbor, draw line connecting origional v to it
        for n in neighbors:
            set_stroke_color(1, 0, 0)
            draw_line(vert[0], vert[1], n[0], n[1])

    #change the fill color for visual astetics
    set_stroke_color(0, 0, 0)
    set_fill_color(0, 1, 0)

    #draw all the shapes using their coordinates
    for shape in coordinates:
        draw_rectangle(shape[0][0], shape[0][1], shape[3][0]-shape[0][0], shape[3][1]-shape[0][1])

    #change fill color to visualize points
    set_fill_color(0, 0, 0)

    #for each location in the path the rrt calculated, draw a circle with 5 radius
    for index in (0, k-1):
        draw_circle(path[index][0], path[index][1], 5)
    if k < len(path):
        k+=1

#begin those graphics
start_graphics(draw, width=500, height= 500)