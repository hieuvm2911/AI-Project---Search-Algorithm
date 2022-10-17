from array import *
import pygame
from pygame.locals import *
import sys, getopt


red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
black = (0, 0, 0)
white = (255, 255, 255)
light_yellow = (255, 255, 102)
# Initialize pygame
pygame.init()
#create the screen
screen = pygame.display.set_mode((20*35, 20*15))

# fill a cell with a color and draw a border
def fill_cell (x1, y1, x2, y2, color):
    pygame.draw.rect(screen, color, (x1, y1, x2, y2))
    pygame.draw.rect(screen, black, (x1, y1, x2, y2), 1)

#fill screen background with white
screen.fill(white)

def read_maze(file_name):
    maze = []
    with open(file_name, "r") as f:
        for line in f:
            maze.append(line)
    # check the goal in maze (first and last line + first and last column)
    for i in range(len(maze[0])):
        if maze[0][i] == " ":
            maze[0] = maze[0][:i] + "G" + maze[0][i+1:]
    for i in range(len(maze) - 1):
        if maze[len(maze) - 1][i] == " ":
            maze[len(maze) - 1] = maze[len(maze) - 1][:i] + "G" + maze[len(maze) - 1][i+1:]
    for i in range(1, len(maze) - 1):
        if maze[i][0] == " ":
            maze[i] = maze[i][:0] + "G" + maze[i][1:]
        elif maze[i][len(maze[i]) - 1] == " ":
            maze[i] = maze[i][:len(maze[i]) - 1] + "G" + maze[i][len(maze[i]):]

    return maze

# draw the maze
def draw_maze (maze):
    for i in range(len(maze)):
        for j in range(len(maze[i])):
            if maze[i][j] == "X":
                fill_cell(j*20, i*20, 20, 20, blue)
            elif maze[i][j] == "S":
                fill_cell(j*20, i*20, 20, 20, green)
            elif maze[i][j] == " " or maze[i][j] == "G":
                fill_cell(j*20, i*20, 20, 20, white)

#heruistic function using euler distance
def heuristic(current, goal):
    return (float) ((current[0] - goal[0])**2 + (current[1] - goal[1])**2)**0.5

#find the start and goal position
def find_start_goal(maze):
    for i in range(len(maze)):
        for j in range(len(maze[i])):
            if maze[i][j] == "S":
                start = (i, j)
            elif maze[i][j] == "G":
                goal = (i, j)
    return start, goal

#find the neighbors of a cell
def find_neighbors(maze, current):
    neighbors = []
    if current[0] > 0:
        if maze[current[0] - 1][current[1]] != "X":
            neighbors.append((current[0] - 1, current[1]))
    if current[0] < len(maze) - 1:
        if maze[current[0] + 1][current[1]] != "X":
            neighbors.append((current[0] + 1, current[1]))
    if current[1] > 0:
        if maze[current[0]][current[1] - 1] != "X":
            neighbors.append((current[0], current[1] - 1))
    if current[1] < len(maze[0]) - 1:
        if maze[current[0]][current[1] + 1] != "X":
            neighbors.append((current[0], current[1] + 1))
    return neighbors

#find the path from start to goal using dfs
def dfs(maze):
    start, goal = find_start_goal(maze)
    frontier = []
    frontier.append(start)
    came_from = {}
    came_from[start] = None
    while len(frontier) > 0:
        current = frontier.pop()
        if current == goal:
            fill_cell(current[1]*20, current[0]*20, 20, 20, red)
            break
        for next in find_neighbors(maze, current):
            if next not in came_from:
                frontier.append(next)
                fill_cell(next[1]*20, next[0]*20, 20, 20, light_yellow)
                pygame.display.update()
                pygame.time.wait()
                came_from[next] = current
    return came_from

#find the path from start to goal using greedy best first search
def greedy_best_first_search(maze):
    start, goal = find_start_goal(maze)
    frontier = []
    frontier.append(start)
    came_from = {}
    came_from[start] = None
    while len(frontier) > 0:
        current = frontier[0]
        for i in range(len(frontier)):
            if heuristic(frontier[i], goal) < heuristic(current, goal):
                current = frontier[i]
        if current == goal:
            fill_cell(current[1]*20, current[0]*20, 20, 20, red)
            break
        frontier.remove(current)
        for next in find_neighbors(maze, current):
            if next not in came_from:
                frontier.append(next)
                fill_cell(next[1]*20, next[0]*20, 20, 20, light_yellow)
                pygame.display.update()
                pygame.time.wait(60)
                came_from[next] = current
    return came_from

#find the path from start to goal using A* search
def a_star_search(maze):
    start, goal = find_start_goal(maze)
    frontier = []
    frontier.append(start)
    came_from = {}
    came_from[start] = None
    cost = {}
    cost[start] = 0
    while len(frontier) > 0:
        current = frontier[0]
        for i in range(len(frontier)):
            if cost[frontier[i]] + heuristic(frontier[i], goal) < cost[current] + heuristic(current, goal):
                current = frontier[i]
        if current == goal:
            fill_cell(current[1]*20, current[0]*20, 20, 20, red)
            break
        frontier.remove(current)
        for next in find_neighbors(maze, current):
            new_cost = cost[current] + 1
            if next not in cost or new_cost < cost[next]:
                cost[next] = new_cost
                frontier.append(next)
                fill_cell(next[1]*20, next[0]*20, 20, 20, light_yellow)
                pygame.display.update()
                pygame.time.wait(60)
                came_from[next] = current
    return came_from

#draw the path from start to goal
def draw_path(maze, came_from):
    start, goal = find_start_goal(maze)
    current = goal
    while current != start:
        current = came_from[current]
        fill_cell(current[1]*20, current[0]*20, 20, 20, green)
        pygame.display.update()
        pygame.time.wait(100)

#write path to output file
def write_path(maze, came_from, num_file):
    start, goal = find_start_goal(maze)
    output_maze = maze
    current = goal
    while True:
        current = came_from[current]
        if current == start:
            break
        output_maze[current[0]] = output_maze[current[0]][:current[1]] + "*" + output_maze[current[0]][current[1]+1:]
    output = open("output" + num_file + ".txt", "w")
    for i in range(len(output_maze)):
        output.write(output_maze[i])
    output.close()

def main():
    if sys.argv[1] == "1":
        maze = read_maze("input1.txt")
    elif sys.argv[1] == "2":
        maze = read_maze("input2.txt")
    
    draw_maze(maze)
    if sys.argv[2] == "gbfs":
        came_from = greedy_best_first_search(maze)
    elif sys.argv[2] == "astar":
        came_from = a_star_search(maze)
    elif sys.argv[2] == "dfs":
        came_from = dfs(maze)
    draw_path(maze, came_from)
    write_path(maze, came_from, sys.argv[1])
    pygame.display.update()
    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

if __name__ == "__main__":
    main()
