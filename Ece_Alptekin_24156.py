from math import sqrt
import time
import profile
from memory_profiler import profile

class Node:
    
    def __init__(self, coordinate, pr=[], c=0, gcost=0):
           
        self.coordinate = coordinate
        self.parent     = pr
        self.cost       = c
        self.gcost      = gcost
          
        if pr is not None:
            self.parent = pr
        else:
            self.cost = 0
    
    def setParent(self, parent):
        self.parent = parent
        
    def getParent(self):
        return self.parent

    def setCost(self, cost):
        self.cost = cost
        
    def getCost(self):
        return self.cost
    
    def getGcost(self):
        return self.gcost
        
    def getCoordinate(self):
        return self.coordinate
    
def Queue(queue, node):
#add node to the queue and return updated queue

    if(len(queue) == 0):
        queue.append(node)
    
    else:
        cost      = node.getCost()
        firstcost = queue[0].getCost()
        lastcost  = queue[len(queue)-1].getCost()
        
        if(cost > lastcost):
            queue.append(node)
        elif(cost < firstcost):
            queue.insert(0, node)
        else:
            i=0
            for item in queue:
                if(cost > item.cost):
                    i += 1
            queue.insert(i, node)
            
    return queue
        
def SearchInQueue(queue, item):
#if node is in queue, return index
#if node is not in queue, return None

    index=0
    for x in queue:
        if(x.coordinate == item.coordinate):
            return index
        index += 1
    return None

def checkEmptyCoordinate(x, y, empty_tiles):
#if (x,y) is a X tile, return true

    for tile in empty_tiles:
        if((x, y) == tile):
            return True
    return False
    
def checkValidCoordinate(x, y, empty_tiles, mSize, nSize):

    #out of bound
    if(x >= mSize):
        return False

    #out of bound
    elif(y >= nSize):
        return False
    #out of bound
    elif(x < 0):
        return False
    #out of bound
    elif(y < 0):
        return False

    #x tile
    elif(checkEmptyCoordinate(x, y, empty_tiles)):
        return False
    
    else:
        return True



def getNeighbor(currentNode, empty_tiles, mSize, nSize):

    neighbor=[]

    if(len(currentNode.coordinate) == 1):
        x = currentNode.coordinate[0][0]
        y = currentNode.coordinate[0][1]

        u = list([list([x - 2, y]), list([x - 1, y])])
        r = list([list([x, y + 1]), list([x, y + 2])])
        l = list([list([x, y - 2]), list([x, y - 1])])
        d = list([list([x + 1, y]), list([x + 2, y])])

        if (checkValidCoordinate(x-2, y, empty_tiles, mSize, nSize) and checkValidCoordinate(x-1, y, empty_tiles, mSize, nSize)):

            up = Node(u, [], 0, 0)
            neighbor.append(up)
        if (checkValidCoordinate(x, y+1, empty_tiles, mSize, nSize) and checkValidCoordinate(x, y+2, empty_tiles, mSize, nSize)):

            right = Node(r, [], 0, 0)
            neighbor.append(right)
        if (checkValidCoordinate(x, y-2, empty_tiles, mSize, nSize) and checkValidCoordinate(x, y-1,empty_tiles, mSize, nSize)):

            left = Node(l, [], 0, 0)
            neighbor.append(left)
        if (checkValidCoordinate(x+1, y, empty_tiles, mSize, nSize) and checkValidCoordinate(x+2, y, empty_tiles, mSize, nSize)):

            down = Node(d, [], 0, 0)
            neighbor.append(down)

        return neighbor

    elif(len(currentNode.coordinate) == 2):
        x1 = currentNode.coordinate[0][0]
        y1 = currentNode.coordinate[0][1]
        x2 = currentNode.coordinate[1][0]
        y2 = currentNode.coordinate[1][1]

        if(x1 == x2):

            u = list([list([x1 - 1, y1]), list([x2 - 1, y2])])
            r = [list([x2, y2 + 1])]
            l = [list([x1, y1 - 1])]
            d = list([list([x1 + 1, y1]), list([x2 + 1, y2])])

            if (checkValidCoordinate(x1-1, y1, empty_tiles, mSize, nSize) and checkValidCoordinate(x2-1, y2, empty_tiles, mSize, nSize)):

                up = Node(u, [], 0, 0)
                neighbor.append(up)
            if (checkValidCoordinate(x2, y2+1, empty_tiles, mSize, nSize)):

                right = Node(r, [], 0, 0)
                neighbor.append(right)
            if (checkValidCoordinate(x1, y1-1, empty_tiles, mSize, nSize)):

                left = Node(l, [], 0, 0)
                neighbor.append(left)
            if (checkValidCoordinate(x1 + 1, y1, empty_tiles, mSize, nSize) and checkValidCoordinate(x2 + 1, y2, empty_tiles, mSize, nSize)):

                down = Node(d, [], 0, 0)
                neighbor.append(down)

            return neighbor

        else:

            u = [list([x1 - 1, y1])]
            r = list([list([x1, y1 + 1]), list([x2, y2 + 1])])
            l = list([list([x1, y1 - 1]), list([x2, y2 - 1])])
            d = [list([x2 + 1, y2])]

            if (checkValidCoordinate(x1 - 1, y1, empty_tiles, mSize, nSize)):

                up = Node(u, [], 0, 0)
                neighbor.append(up)
            if (checkValidCoordinate(x1, y1 + 1, empty_tiles, mSize, nSize) and checkValidCoordinate(x2, y2+1, empty_tiles, mSize, nSize)):

                right = Node(r, [], 0, 0)
                neighbor.append(right)
            if (checkValidCoordinate(x1, y1 - 1, empty_tiles, mSize, nSize) and checkValidCoordinate(x2, y2-1, empty_tiles, mSize, nSize)):

                left = Node(l, [], 0, 0)
                neighbor.append(left)
            if (checkValidCoordinate(x2+1, y2, empty_tiles, mSize, nSize)):

                down = Node(d, [], 0, 0)
                neighbor.append(down)

            return neighbor

        

def HeuristicFunction(goal, current):
    
    goalx = goal.coordinate[0][0]
    goaly = goal.coordinate[0][1]
    
    if(len(current.coordinate) == 2):
        x = (current.coordinate[0][0] + current.coordinate[1][0])/2
        y = (current.coordinate[0][1] + current.coordinate[1][1])/2
    else:
        x = current.coordinate[0][0]
        y = current.coordinate[0][1]

    #Euclidean distance
    return sqrt((goalx - x) ** 2 + (goaly - y) ** 2)


@profile(precision=4)
def ASTAR(goal, start, empty_tiles, mSize, nSize):    
    
    frontier = []  
    closed   = []      
    start_node = Node(start.getCoordinate(), [], 0)
    frontier   = Queue(frontier, start_node)                              
    
    while len(frontier) != 0:
        
        current = frontier.pop(0)
        parent  = current.getParent()
        parent  = parent + [current.getCoordinate()]

        #the game is ended
        if current.coordinate == goal.coordinate:
            return parent
        
        currentCoord = current.getCoordinate()
        currentCost  = current.getCost()

        #neighbors of the current node
        neighbor = getNeighbor(current, empty_tiles, mSize, nSize) 
        succ     = []
        
        for x in neighbor:
            coordinate   = x.getCoordinate()
            gcost        = 1 + current.getGcost()
            cost         = HeuristicFunction(goal, x) + gcost 
            node         = Node(coordinate, parent, cost, gcost)
            succ.append(node) 

        for item in succ:
            cost           = item.getCost()             
            index_frontier = SearchInQueue(frontier, item)    
            index_closed   = SearchInQueue(closed, item)

            #node is not in frontier and node is not in closed
            if(index_frontier == None and index_closed == None):   
                frontier = Queue(frontier, item)

            #node is in frontier and the cost is bigger
            elif(index_frontier != None and frontier[index_frontier].getCost() > cost):    
                frontier[index_frontier].setCost(cost)
                item_parent = item.getParent()
                frontier[index_frontier].setParent(item_parent)

            #node is in closed and the cost is bigger
            elif(index_closed != None and closed[index_closed].getCost() > cost):
                frontier = Queue(frontier, item)

        #add node to the closed
        nd     = Node(currentCoord, None, currentCost)
        closed = Queue(closed, nd)
        
    return None




@profile(precision=4)
def UCS(goal, start, empty_tiles, mSize, nSize):
    
    frontier = []  
    closed   = []      
    start_node = Node(start.getCoordinate(), [], 0)
    frontier   = Queue(frontier, start_node)
    
    while len(frontier) != 0:
        
        current = frontier.pop(0)
        parent  = current.getParent()
        parent  = parent + [current.getCoordinate()]

        #the game is ended
        if current.coordinate == goal.coordinate:
            return parent
        
        currentCoord = current.getCoordinate()
        currentCost  = current.getCost()

        #add node to the closed
        nd     = Node(currentCoord, None, currentCost)
        closed = Queue(closed, nd)

        #neighbors of the current node
        neighbor = getNeighbor(current, empty_tiles, mSize, nSize)
        succ     = []

        for x in neighbor:
            coordinate   = x.getCoordinate()
            cost         = currentCost + 1
            temp_node    = Node(coordinate, parent, cost)
            succ.append(temp_node)
                                  
        for item in succ:
            cost  = item.getCost()             
            index_frontier = SearchInQueue(frontier, item)    
            index_closed   = SearchInQueue(closed, item)
            
            #node is not in frontier and node is not in closed
            if(index_frontier == None and index_closed == None):   
                frontier = Queue(frontier, item)

            #node is in frontier and the cost is bigger
            elif(index_frontier != None and frontier[index_frontier].getCost() > cost):    
                frontier[index_frontier].setCost(cost)
                item_parent = item.getParent()
                frontier[index_frontier].setParent(item_parent)
                
                
    return None
















start_coordinate = []
goal_coordinate  = []  
empty_tiles      = []


i=0       
filename = input("Please enter the filename for the game board: ")
file     = open(filename, mode='r')


for line in iter(file.readline, ''):

    x      = line.replace(" ", "")
    length = len(x)
    row    = list(x[0:length])
    
    column = 0    
    for char in row:
        if char == 'G':
            goal_y = x.index('G')
            goal_coordinate.append((i, goal_y))          
        elif char == 'S':
            start_coordinate.append((i, column))
        elif char == 'X':
            empty_tiles.append((i, column))
            
        column += 1
    nSize  = column
 
    i += 1
    
mSize = i

file.close()


if(len(start_coordinate) == 2):
    x1 = start_coordinate[0][0]
    y1 = start_coordinate[0][1]
    x2 = start_coordinate[1][0]
    y2 = start_coordinate[1][1]
    startCoordinate = list([list([x1, y1]), list([x2, y2])])
    start = Node(startCoordinate , [], 0)
else:
    x1 = start_coordinate[0][0]
    y1 = start_coordinate[0][1]
    startCoordinate = [list([x1, y1])]
    start = Node(startCoordinate , [], 0)

goal_x = goal_coordinate[0][0]
goal_y = goal_coordinate[0][1]
goalCoordinate = [list([goal_x, goal_y])]
goal = Node(goalCoordinate, [], 0)

StartTime = time.time()    
    
ucs = UCS(goal, start, empty_tiles, mSize, nSize)

if(ucs == None):
    EndTime = time.time()  
    print("UCS")
    print("No possible path")
    print("Duration:", EndTime - StartTime)
        
elif(ucs != None):
    EndTime = time.time()
    print("UCS")
    print("Duration:", EndTime - StartTime)
    for x in ucs:
        print(x)
        
starttime = time.time() 
        
astar = ASTAR(goal, start, empty_tiles, mSize, nSize)

if(astar == None):
    endtime = time.time() 
    print("ASTAR")
    print("No possible path")
    print("Duration:", endtime - starttime)
        
elif(astar != None):
    endtime = time.time()
    print("ASTAR")
    print("Duration:", endtime - starttime)
    for x in astar:
        print(x)


