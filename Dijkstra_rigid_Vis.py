import numpy as np
import cv2

try:
    radius = int(input('Enter radius of the robot: '))
    if radius < 0:
        print("Invalid radius, setting radius to 0")
        radius = 0

    clearance = int(input('Enter clearance: '))
    if clearance < 0:
        print("Invalid clearance, setting clearance to 0")
        clearance = 0

except:
    print("Error: Invalid Input. Exiting program")
    exit(3)

cl = radius + clearance

# Map creation with edges as '1' in order to provide a void border of the map

obs_map = np.zeros((302, 402), dtype=int)
obs_map[0, :] = 1
obs_map[301, :] = 1
obs_map[:, 0] = 1
obs_map[:, 401] = 1


class Queue:

    # Creating a class to convert a list into a queue

    def __init__(self):
        self.queue = []

    def add(self, node):
        self.queue.append(node)

    def pop(self):
        ind = self.queue.index(min(self.queue))
        node = self.queue.pop(ind)
        return node

    def __len__(self):
        return len(self.queue)


# Creating a class to determine the node of the iteration. Node is the puzzle state.

class Node:

    # Defining the __init__ function

    def __init__(self, data, parent, act, cost):
        self.data = data
        self.parent = parent
        self.act = act
        self.id = self.get_id()
        self.cost = cost

    def __eq__(self, other):
        if hasattr(other, 'cost'):
            return self.cost == other.cost
        else:
            raise NotImplementedError('Not supported between given types')

    def __ne__(self, other):
        if hasattr(other, 'cost'):
            return self.cost != other.cost
        else:
            raise NotImplementedError('Not supported between given types')

    def __lt__(self, other):
        if hasattr(other, 'cost'):
            return self.cost < other.cost
        else:
            raise NotImplementedError('Not supported between given types')

    def __gt__(self, other):
        if hasattr(other, 'cost'):
            return self.cost > other.cost
        else:
            raise NotImplementedError('Not supported between given types')

    def __le__(self, other):
        if hasattr(other, 'cost'):
            return self.cost <= other.cost
        else:
            raise NotImplementedError('Not supported between given types')

    def __ge__(self, other):
        if hasattr(other, 'cost'):
            return self.cost >= other.cost
        else:
            raise NotImplementedError('Not supported between given types')

    # Defining a function to generate a unique id of the state of the puzzle.

    def get_id(self):
        _id = np.ravel(self.data).tolist()
        _id = [str(item) for item in _id]
        _id = "-".join(_id)
        self.id = _id
        return self.id

    # Defining the __repr__ function

    def __repr__(self):
        return str(self.data)


# Creating a function to define the circle obstacle's area on the map

def getCircleObstacle(i, j):
    global cl
    cond = ((j - 90) ** 2) + ((i - 70) ** 2) <= ((35 + cl) ** 2)
    return cond


# Creating a function to define the C shape obstacle's area on the map

def getCShapeObstacle(i, j):
    global cl
    cond1 = i <= 270 - cl
    cond2 = i <= 280 + cl
    cond3 = j >= 200 - cl
    cond4 = j >= 210 + cl
    cond5 = i >= 240 + cl
    cond6 = i >= 230 - cl
    cond7 = j <= 230 + cl
    ret_val = ((cond2 and cond3 and cond6 and cond7) and not (cond1 and cond4 and cond5 and cond7))
    return ret_val


# Creating a function to define the slanted rectangle obstacle's area on the map

def getSlantedRectObstacle(i, j):
    global cl
    s1 = 0.7
    s2 = -1.42814
    x1 = np.arctan(s1)
    x2 = np.arctan(s2)
    d1 = np.cos(np.pi - x1)
    d2 = np.cos(np.pi - x2)
    a = -(cl / d1)
    b = -(cl / d2)
    cond1 = (i) + (1.42814 * j) >= (176.5511 - b)
    cond2 = (i) - (0.7 * j) >= (74.39 - a)
    cond3 = (i) + (1.42814 * j) <= (428.06815 + b)
    cond4 = (i) - (0.7 * j) <= (98.80545 + a)
    ret_val = (cond1 and cond2 and cond3 and cond4)
    return ret_val

# Creating a function to define the ellipse obstacle's area on the map

def getEllipseObstacle(i, j):
    global cl
    cond = (((j - 246) / (60 + cl)) ** 2) + (((i - 145) / (30 + cl)) ** 2) <= 1
    return cond

def getBorderClearance(i, j):
    global cl
    cond1 = j >= 402 - cl
    cond2 = j <= cl
    cond3 = i >= 302 - cl
    cond4 = i <= cl
    ret_val = cond1 or cond2 or cond3 or cond4
    return ret_val


# Creating a function to define the polygon obstacle's area on the map
# The Polygon is divided into a rectangle and two triangles


# Creating an if-condition to change value of the element in area under all obstacles to '1' in order to create a void in the map

for i in range(obs_map.shape[0]):
    for j in range(obs_map.shape[1]):
        if getCircleObstacle(obs_map.shape[0] - i, j) or getCShapeObstacle(obs_map.shape[0] - i,
                                                                           j) or getSlantedRectObstacle(
            obs_map.shape[0] - i, j) or getEllipseObstacle(obs_map.shape[0] - i, j) or getBorderClearance(
            obs_map.shape[0] - i, j):
            obs_map[i, j] = 1


# Defining the move up function where if the element above does not have '1' value, i.e. if there isn't a void in the element above, the object moves up

def move_up(i, j):
    if obs_map[i - 1, j] != 1:
        return (i - 1, j)


# Defining the move down function where if the element below does not have '1' value, i.e. if there isn't a void in the element below, the object moves down

def move_down(i, j):
    if obs_map[i + 1, j] != 1:
        return (i + 1, j)


# Defining the move left function where if the element on the left does not have '1' value, i.e. if there isn't a void in the element on the left, the object moves left

def move_left(i, j):
    if obs_map[i, j - 1] != 1:
        return (i, j - 1)


# Defining the move right function where if the element on the right does not have '1' value, i.e. if there isn't a void in the element on the right, the object moves right

def move_right(i, j):
    if obs_map[i, j + 1] != 1:
        return (i, j + 1)


# Defining the move up left function where if the element above and left does not have '1' value, i.e. if there isn't a void in the element above and left , the object moves up left

def move_up_left(i, j):
    if obs_map[i - 1, j - 1] != 1:
        return (i - 1, j - 1)


# Defining the move up right function where if the element above and right does not have '1' value, i.e. if there isn't a void in the element above and right, the object moves up right

def move_up_right(i, j):
    if obs_map[i - 1, j + 1] != 1:
        return (i - 1, j + 1)


# Defining the move down left function where if the element below and left does not have '1' value, i.e. if there isn't a void in the element below and left, the object moves down left

def move_down_left(i, j):
    if obs_map[i + 1, j - 1] != 1:
        return (i + 1, j - 1)


# Defining the move down right function where if the element below and right does not have '1' value, i.e. if there isn't a void in the element below and right, the object moves down right

def move_down_right(i, j):
    if obs_map[i + 1, j + 1] != 1:
        return (i + 1, j + 1)


# Defining a function to generate new legal moves as per the state

def generate_new_moves(state):
    list_states = []
    for func in [move_left, move_right, move_down, move_up]:
        cost = state.cost + 1
        dum_state = state.data
        out_state = func(dum_state[0], dum_state[1])
        if out_state is not None:
            list_states.append((out_state, cost))
    for func in [move_up_left, move_up_right, move_down_left,
                 move_down_right]:
        cost = state.cost + 1.414
        dum_state = state.data
        out_state = func(dum_state[0], dum_state[1])
        if out_state is not None:
            list_states.append((out_state, cost))
    return list_states


# Inputting values from the user and checking if the values are valid by checking the outbound values and in-obstacle values

try:
    start_node_x = int(input('Enter start node x postion: '))
    if start_node_x < 0:
        print("Invalid start node x position, setting x postion to 0")
        start_node_x = 0
    elif start_node_x > 402:
        print("Invalid start node x position, setting x postion to 403")
        start_node_x = 402

    start_node_y = int(input('Enter start node y postion: '))
    if start_node_y < 0:
        print("Invalid start node y position, setting y postion to 0")
        start_node_y = 0
    elif start_node_y > 302:
        print("Invalid start node y position, setting y postion to 300")
        start_node_y = 302

    goal_node_x = int(input('Enter goal node x postion: '))
    if goal_node_x < 0:
        print("Invalid goal node x position, setting x postion to 0")
        goal_node_x = 0
    elif goal_node_x > 402:
        print("Invalid goal node x position, setting x postion to 403")
        start_node_x = 402

    goal_node_y = int(input('Enter goal node y postion: '))
    if goal_node_y < 0:
        print("Invalid goal node y position, setting y postion to 0")
        goal_node_y = 0
    elif goal_node_y > 302:
        print("Invalid goal node y position, setting y postion to 300")
        start_node_y = 302

    if obs_map[obs_map.shape[0] - start_node_y, start_node_x] == 1:
        print("Error: Start position is in void space. Exiting program")
        exit(1)

    if obs_map[obs_map.shape[0] - goal_node_y, goal_node_x] == 1:
        print("Error: Goal position is in void space. Exiting program")
        exit(1)
except:
    print("Error: Invalid Input. Exiting program")
    exit(2)

# Creating the goal state and initial state.

goal_state = (obs_map.shape[0] - goal_node_y, goal_node_x)
init_state = (obs_map.shape[0] - start_node_y, start_node_x)
state_queue = Queue()
state_queue.add(Node(init_state, None, None, 0))

visited = []

# Creating a new array in order to write as video

result_map = obs_map.copy()
result_map = result_map * 255
result_map = np.dstack((result_map, result_map, result_map))
result_map = result_map.astype(np.uint8)
height, width = obs_map.shape
FPS_val = 240

video_save = cv2.VideoWriter("Path_Detection.mp4", cv2.VideoWriter_fourcc(*'mp4v'), FPS_val, (width, height))

# While loop to iterate the values inside the array with legal moves.
# If the current state is same as the goal state then the loop breaks.
# If the state ID is found in the visited list, then the node is skipped

def ClearanceSpace(i, j):
    global clearance
    cl = clearance
    s1 = 0.7
    s2 = -1.42814
    x1 = np.arctan(s1)
    x2 = np.arctan(s2)
    d1 = np.cos(np.pi - x1)
    d2 = np.cos(np.pi - x2)
    a = -(cl / d1)
    b = -(cl / d2)
    ellipse1 = (((j - 246) / 60) ** 2) + (((i - 145) / 30) ** 2) <= 1
    ellipse2 = (((j - 246) / (60 + cl)) ** 2) + (((i - 145) / (30 + cl)) ** 2) <= 1
    circle1 = ((j - 90) ** 2) + ((i - 70) ** 2) <= ((35) ** 2)
    circle2 = ((j - 90) ** 2) + ((i - 70) ** 2) <= ((35 + cl) ** 2)
    rect1 = (i) + (1.42814 * j) >= (176.5511)
    rect2 = (i) - (0.7 * j) >= (74.39)
    rect3 = (i) + (1.42814 * j) <= (428.06815)
    rect4 = (i) - (0.7 * j) <= (98.80545)
    rect5 = (i) - (s1 * j) <= (98.80545 + a)
    rect6 = (i) - (s2 * j) >= (176.5511 - b)
    rect7 = (i) - (s1 * j) >= (74.39 - a)
    rect8 = (i) - (s2 * j) <= (428.06815 + b)
    cshape1 = i <= 270
    cshape2 = i <= 280
    cshape3 = j >= 200
    cshape4 = j >= 210
    cshape5 = i >= 240
    cshape6 = i >= 230
    cshape7 = j <= 230
    cshape8 = i <= 270 - cl
    cshape9 = i <= 280 + cl
    cshape10 = j >= 200 - cl
    cshape11 = j >= 210 + cl
    cshape12 = i >= 240 + cl
    cshape13 = i >= 230 - cl
    cshape14 = j <= 230 + cl
    cshapecond1 = (cshape9 and cshape10 and cshape13 and cshape14) and not (cshape8 and cshape11 and cshape12 and cshape14)
    cshapecond2 = (cshape2 and cshape3 and cshape6 and cshape7) and not (cshape1 and cshape4 and cshape5 and cshape7)
    bord1 = j >= 402 - cl
    bord2 = j <= cl
    bord3 = i >= 302 - cl
    bord4 = i <= cl
    ellipsecond = (ellipse2 and not ellipse1)
    circlecond = (circle2 and not circle1)
    rectcond = ((rect5 and rect6 and rect7 and rect8) and not (rect1 and rect2 and rect3 and rect4))
    cshapecond = ((cshapecond1) and not (cshapecond2))
    bordcond = (bord1 or bord2 or bord3 or bord4)
    ret_val = (ellipsecond or circlecond or rectcond or cshapecond or bordcond)
    #ret_val = cshapecond
    return ret_val


def Blackout(i, j):
    global clearance, radius
    r = radius + clearance
    cl = clearance
    s1 = 0.7
    s2 = -1.42814
    x1 = np.arctan(s1)
    x2 = np.arctan(s2)
    d1 = np.cos(np.pi - x1)
    d2 = np.cos(np.pi - x2)
    a = -(cl / d1)
    b = -(cl / d2)
    c = -(r / d1)
    d = -(r / d2)
    ellipse1 = (((j - 246) / (60 + r)) ** 2) + (((i - 145) / (30 + r)) ** 2) <= 1
    ellipse2 = (((j - 246) / (60 + cl)) ** 2) + (((i - 145) / (30 + cl)) ** 2) <= 1
    circle1 = ((j - 90) ** 2) + ((i - 70) ** 2) <= ((35 + r) ** 2)
    circle2 = ((j - 90) ** 2) + ((i - 70) ** 2) <= ((35 + cl) ** 2)
    rect1 = (i) + (1.42814 * j) >= (176.5511 - d)
    rect2 = (i) - (0.7 * j) >= (74.39 - c)
    rect3 = (i) + (1.42814 * j) <= (428.06815 + d)
    rect4 = (i) - (0.7 * j) <= (98.80545 + c)
    rect5 = (i) - (s1 * j) <= (98.80545 + a)
    rect6 = (i) - (s2 * j) >= (176.5511 - b)
    rect7 = (i) - (s1 * j) >= (74.39 - a)
    rect8 = (i) - (s2 * j) <= (428.06815 + b)
    cshape1 = i <= 270 - cl
    cshape2 = i <= 280 + cl
    cshape3 = j >= 200 - cl
    cshape4 = j >= 210 + cl
    cshape5 = i >= 240 + cl
    cshape6 = i >= 230 - cl
    cshape7 = j <= 230 + cl
    cshape8 = i <= 270 - r
    cshape9 = i <= 280 + r
    cshape10 = j >= 200 - r
    cshape11 = j >= 210 + r
    cshape12 = i >= 240 + r
    cshape13 = i >= 230 - r
    cshape14 = j <= 230 + r
    bord1 = j <= 401 - cl
    bord2 = j >= cl
    bord3 = i <= 301 - cl
    bord4 = i >= cl
    bord5 = j <= 401 - r
    bord6 = j >= r + 1
    bord7 = i <= 301 - r
    bord8 = i >= r + 1
    cshapecond1 = (cshape9 and cshape10 and cshape13 and cshape14) and not (cshape8 and cshape11 and cshape12 and cshape14)
    cshapecond2 = (cshape2 and cshape3 and cshape6 and cshape7) and not (cshape1 and cshape4 and cshape5 and cshape7)
    cshapecond = (cshapecond1 and not cshapecond2)
    ellipsecond = (ellipse1 and not ellipse2)
    circlecond = (circle1 and not circle2)
    rectcond = ((rect1 and rect2 and rect3 and rect4) and not (rect5 and rect6 and rect7 and rect8))
    bordcond = ((bord1 and bord2 and bord3 and bord4) and not (bord5 and bord6 and bord7 and bord8))
    ret_val = (ellipsecond or circlecond or rectcond or cshapecond or bordcond)
    return ret_val


for i in range(result_map.shape[0]):
    for j in range(result_map.shape[1]):
        if ClearanceSpace(result_map.shape[0] - i, j):
            result_map[i, j] = np.asarray([0, 0, 0])

for i in range(result_map.shape[0]):
    for j in range(result_map.shape[1]):
        if Blackout(result_map.shape[0] - i, j):
            result_map[i, j] = np.asarray([0, 0, 0])

while True:
    try:
        cur_node = state_queue.pop()
    except:
        if len([node for node in state_queue.queue]) == 0:
            break
    if np.all(cur_node.data == goal_state):
        break
    if cur_node.id in visited:
        continue
    moves = generate_new_moves(cur_node)
    for move in moves:
        new_node = Node(move[0], cur_node, None, move[1])
        state_queue.add(new_node)
    visited.append(cur_node.id)

    result_map[cur_node.data[0], cur_node.data[1], :] = np.asarray((255, 0, 0))

    result_map[result_map.shape[0] - start_node_y, start_node_x, :] = np.asarray((0, 0, 255))
    result_map[result_map.shape[0] - goal_node_y, goal_node_x, :] = np.asarray((0, 255, 0))

    video_save.write(result_map)

target_node = cur_node
path = []

# While loop to add a step in the path

while cur_node is not None:
    path.append(cur_node)
    cur_node = cur_node.parent

# Traceback the path

path.reverse()

# Converting the data in path array to BGR values

for item in path:
    result_map[item.data[0], item.data[1], :] = np.asarray((0, 255, 255))
    result_map[result_map.shape[0] - start_node_y, start_node_x, :] = np.asarray((0, 0, 255))
    result_map[result_map.shape[0] - goal_node_y, goal_node_x, :] = np.asarray((0, 255, 0))
    result_map = cv2.circle(result_map, (item.data[1], item.data[0]), radius, (0, 255, 255), -1)

    for _ in range(int(FPS_val / 20)):
        video_save.write(result_map)

# Writing and saving the complete traverse

video_save.write(result_map)

video_save and video_save.release()
cv2.imshow("Path", result_map)
if cv2.waitKey(0) and 0XFF == ord('q'):
    exit(0)

cv2.destroyAllWindows()