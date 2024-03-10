# Importing the library
import pygame
import math
import heapq
import time


start_time = time.time()

# Initializing Pygame
pygame.init()
 
# Initializing surface
surface = pygame.display.set_mode((1200,500))
 
# Initializing Color
color = (112,128,144)
inflation_color = (31, 81, 255)
exploration_color = (220, 220, 220)

inflated_obstacles = []
obstacle = []
map_inflation = []


# function to draw a hexagon from radius and center
def draw_hexagon(center, radius):
    angle = 0
    vertices = []
    for i in range(6):
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        vertices.append((x, y))
        angle += math.pi / 3
    return vertices

# function to draw obstacles
def draw_obstacles():
    rectangles = [[100, 0, 75, 400], [275, 100, 75, 400]]
    c_shape = [(900,50), (1100,50),(1100,450),(900,450),(900,375),(1020,375),(1020,125),(900,125)]


    # draw obstacles
    for i in range(len(rectangles)):
        obstacle.append([(rectangles[i][0], rectangles[i][1]), (rectangles[i][0] + rectangles[i][2], rectangles[i][1]), (rectangles[i][0] + rectangles[i][2], rectangles[i][1] + rectangles[i][3]), (rectangles[i][0], rectangles[i][1] + rectangles[i][3])])
    obstacle.append(draw_hexagon((650, 250), 150))
    obstacle.append(c_shape)
    # take obstacles and draw them
    for i in range(len(obstacle)):
        pygame.draw.polygon(surface, color, obstacle[i])

    # add 5 units inflating the obstacles and save them to the inflated_obstacles list
    for i in range(len(rectangles)):
        inflated_obstacles.append([(rectangles[i][0] - 5, rectangles[i][1] - 5), (rectangles[i][0] + rectangles[i][2] + 5, rectangles[i][1] - 5), (rectangles[i][0] + rectangles[i][2] + 5, rectangles[i][1] + rectangles[i][3] + 5), (rectangles[i][0] - 5, rectangles[i][1] + rectangles[i][3] + 5)])
    inflated_obstacles.append(draw_hexagon((650, 250), 155))
    inflated_obstacles.append([(895,45), (1105,45),(1105,455),(895,455),(895,370),(1015,370),(1015,130),(895,130)])


    map_inflation.append([(5,5), (1190,5), (1190,490), (5,490)])
    # draw inflation
    for i in range(len(inflated_obstacles)):
        pygame.draw.polygon(surface, inflation_color, inflated_obstacles[i])
    # draw map inflation
    for i in range(len(map_inflation)):
        pygame.draw.polygon(surface, inflation_color, map_inflation[i],2)





# class to represent the node
class Node:
    def __init__(self, position, parent=None, cost=float('inf')):
        self.position = position
        self.parent = parent
        self.cost = cost

    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)


# functions to move the point robot in the up, down, left, right, up-right, up-left, down-right, down-left directions
def move_up(node):
    return Node((node.position[0], node.position[1] + 1), node, node.cost + 1)

def move_down(node):
    return Node((node.position[0], node.position[1] - 1), node, node.cost + 1)

def move_left(node):
    return Node((node.position[0] - 1, node.position[1]), node, node.cost + 1)

def move_right(node):
    return Node((node.position[0] + 1, node.position[1]), node, node.cost + 1)

def move_up_right(node):
    return Node((node.position[0] + 1, node.position[1] + 1), node, node.cost + 1.4)

def move_up_left(node):
    return Node((node.position[0] - 1, node.position[1] + 1), node, node.cost + 1.4)

def move_down_right(node):
    return Node((node.position[0] + 1, node.position[1] - 1), node, node.cost + 1.4)

def move_down_left(node):
    return Node((node.position[0] - 1, node.position[1] - 1), node, node.cost + 1.4)


# function to check if point is inside the obstacle space
def is_point_in_obstacle_space(point):
    for obstacle in inflated_obstacles:
        intersect_count = 0
        for i in range(len(obstacle)):
            p1 = obstacle[i]
            p2 = obstacle[(i + 1) % len(obstacle)]
            if point[1] > min(p1[1], p2[1]):
                if point[1] <= max(p1[1], p2[1]):
                    if point[0] <= max(p1[0], p2[0]):
                        if p1[1] != p2[1]:
                            x_intersect = (point[1] - p1[1]) * (p2[0] - p1[0]) / (p2[1] - p1[1]) + p1[0]
                            if p1[0] == p2[0] or point[0] <= x_intersect:
                                intersect_count += 1
        if intersect_count % 2 != 0:
            return True
    return False


# function to check if point is outside the map
def is_point_in_map_inflation(point):
    for i in range(len(map_inflation)):
        if point[0] > map_inflation[i][0][0] and point[0] < map_inflation[i][1][0]:
            if point[1] > map_inflation[i][0][1] and point[1] < map_inflation[i][2][1]:
                return False
    return True


# function to get the neighbors of the current node
def get_neighbors(node):
    neighbors = []
    for neighbor_position, cost in [
        (move_up(node), 1), (move_down(node), 1), (move_left(node), 1), (move_right(node), 1),
        (move_up_right(node), 1.4), (move_up_left(node), 1.4), (move_down_right(node), 1.4), (move_down_left(node), 1.4)
    ]:
        neighbors.append((neighbor_position, cost))
    return neighbors


# function to implement the dijkstra algorithm
def dijkstra_algorithm(initial_point, goal_point):
    open_list = []
    closed_set = set()
    start_node = Node(initial_point, None, 0)
    # use heapq to implement priority queue
    heapq.heappush(open_list, start_node)
    while open_list:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        current_node = heapq.heappop(open_list)
        if current_node.position == goal_point:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]
        else:
            if current_node in closed_set:
                continue
            neighbors = get_neighbors(current_node)
            for neighbor, cost in neighbors:
                if is_point_in_obstacle_space(neighbor.position) or is_point_in_map_inflation(neighbor.position):
                    continue
                if neighbor in closed_set:
                    continue
                if neighbor in open_list:
                    for node in open_list:
                        if node.position == neighbor.position:
                            if node.cost > neighbor.cost:
                                node.cost = neighbor.cost
                                node.parent = neighbor.parent
                else:
                    heapq.heappush(open_list, neighbor)
            closed_set.add(current_node)
            pygame.draw.circle(surface, exploration_color, current_node.position, 1)
            pygame.draw.circle(surface, (0,255,0), initial_point, 5)
            pygame.display.update()
    return None
   

# function to get the initial and goal points from the user
def get_initial_and_goal_points():
    # get initial and goal points from user as a tuple input 
    initial_point = tuple(map(int, input("Enter the initial point: ").split()))
    goal_point = tuple(map(int, input("Enter the goal point: ").split()))
    # check if the initial and goal points are inside the map
    while is_point_in_obstacle_space(initial_point) or is_point_in_map_inflation(initial_point):
        print("The initial point is inside the obstacle space or outside the map. Please enter a valid point.")
        initial_point = tuple(map(int, input("Enter the initial point: ").split()))
    while is_point_in_obstacle_space(goal_point) or is_point_in_map_inflation(goal_point):
        print("The goal point is inside the obstacle space or outside the map. Please enter a valid point.")
        goal_point = tuple(map(int, input("Enter the goal point: ").split()))
    return initial_point, goal_point


def main():
    # start run time
    st = time.time()

    pygame.display.set_caption("Dijkstra Algorithm")

    draw_obstacles()
    initial_point, goal_point = get_initial_and_goal_points()
    print("The initial point is: ", initial_point)
    print("The goal point is: ", goal_point)
    # initial_point= (400,50)
    # goal_point = (450,80)

    pygame.draw.circle(surface, (0,255,0), initial_point, 5)
    pygame.draw.circle(surface, (255,0,0), goal_point, 5)
    pygame.display.update()


    path = dijkstra_algorithm(initial_point, goal_point)
    et = time.time()

    print("The algorithm took  %s seconds to find the path" % (et - st))


    if path:
        for i in range(len(path) - 1):
            pygame.draw.line(surface, (34,139,34), path[i], path[i + 1], 2)
        pygame.display.update()
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
    else:
        print("No path found")

if __name__ == "__main__":
    main()
    pygame.quit()