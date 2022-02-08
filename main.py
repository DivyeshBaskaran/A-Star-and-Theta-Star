import heapq
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import random
import math
from datetime import datetime
import tkinter as tk
import os

def generate_blocklist(x_lim,y_lim):
    blocklist = []
    # add margin blocks to blocklist
    for i in range(1, y_lim+1):
        blocklist.append((0, i))
        blocklist.append((x_lim+1, i))
    for j in range(1, x_lim+1):
        blocklist.append((j, 0))
        blocklist.append((j, y_lim+1))
    # randomly choose 10% of the total cells as blocks
    total_cells = x_lim*y_lim
    blocknum_list = random.sample(range(0, total_cells), math.trunc(0.1*total_cells))
    for blocknum in blocknum_list:
        blocklist.append((blocknum % x_lim, math.trunc(blocknum / x_lim) + 1))
    return blocklist


def generate_start_and_goal(x_lim, y_lim):
    start = (random.randint(1, x_lim+1), random.randint(1, y_lim+1))
    goal = (random.randint(1, x_lim+1), random.randint(1, y_lim+1))
    return start, goal


# h() only works for A*
def h(origin, destination):
    return math.sqrt(2)*min(abs(origin[0]-destination[0]), abs(origin[1]-destination[1])) \
           + max(abs(origin[0]-destination[0]), abs(origin[1]-destination[1])) \
           - min(abs(origin[0]-destination[0]), abs(origin[1]-destination[1]))


# get_direct_distance is the h for theta*
def get_direct_distance(origin, destination):
    x_sqr = math.pow((origin[0]-destination[0]), 2)
    y_sqr = math.pow((origin[1]-destination[1]), 2)
    return math.sqrt(x_sqr+y_sqr)


# start, goal should be like (x,y)
def aStar(start, goal, blocklist, x_lim, y_lim):
    # elem in fringe: (f,(x,y))
    fringe = []
    # g is the physical distance from n to start
    # g is dict, the key should be (x,y) and value should be int
    g = {}
    # parent record a positions parent, key and value are both (x,y)
    parent = {}
    g[start] = 0
    parent[start] = start
    heapq.heappush(fringe, (g[start] + h(start, goal), start))
    # closed only store position
    closed = []
    while len(fringe) != 0:
        s = heapq.heappop(fringe)
        if s[1] == goal:
            return "Path Found", parent, g
        closed.append(s[1])
        for s_p in succ(s[1], blocklist, x_lim, y_lim):
            if s_p[0] not in closed:
                if not fringe_has_position(fringe, s_p[0]):
                    g[s_p[0]] = float('inf')
                    parent[s_p[0]] = None
                update_vertex(s, s_p, fringe, g, parent, goal)
    return "No path", parent, g

def update_vertex(s, s_p, fringe, g, parent, goal):
    if g[s[1]] + s_p[1] < g[s_p[0]]:
        g[s_p[0]] = g[s[1]] + s_p[1]
        parent[s_p[0]] = s[1]
        if fringe_has_position(fringe, s_p[0]):
            fringe_remove(fringe, s_p[0])
        heapq.heappush(fringe, (g[s_p[0]] + h(s_p[0], goal), s_p[0]))


def thetaStar(start, goal, blocklist, x_lim, y_lim):
    # elem in fringe: (f,(x,y))
    fringe = []
    # g is the physical distance from n to start
    # g is dict, the key should be (x,y) and value should be int
    g = {}
    # parent record a positions parent, key and value are both (x,y)
    parent = {}
    g[start] = 0
    parent[start] = start
    heapq.heappush(fringe, (g[start] + get_direct_distance(start, goal), start))
    # closed only store position
    closed = []
    while len(fringe) != 0:
        s = heapq.heappop(fringe)
        if s[1] == goal:
            return "Path Found", parent, g
        closed.append(s[1])
        for s_p in succ(s[1], blocklist, x_lim, y_lim):
            if s_p[0] not in closed:
                if not fringe_has_position(fringe, s_p[0]):
                    g[s_p[0]] = float('inf')
                    parent[s_p[0]] = None
                update_vertex2(s, s_p, fringe, g, parent, goal, blocklist)
    return "No path", parent, g

# s is a tuple of (f, (x, y))
# s_p is a tuple of ((x,y), distance between(s, s'))
# g is the path cost dict
# parent is the parent dict for each node
# goal is a tuple of (x, y)
def update_vertex2(s, s_p, fringe, g, parent, goal, blocklist):
    if line_of_sight(parent[s[1]], s_p[0], blocklist):
        # Path 2
        if g[parent[s[1]]] + get_direct_distance(parent[s[1]], s_p[0]) < g[s_p[0]]:
            g[s_p[0]] = g[parent[s[1]]] + get_direct_distance(parent[s[1]], s_p[0])
            parent[s_p[0]] = parent[s[1]]
            if fringe_has_position(fringe, s_p[0]):  # need to fix open list stuff
                fringe_remove(fringe, s_p[0])
            heapq.heappush(fringe, (g[s_p[0]] + get_direct_distance(s_p[0], goal), s_p[0]))
    else:
        # Path 1
        if g[s[1]] + get_direct_distance(s[1], s_p[0]) < g[s_p[0]]:
            g[s_p[0]] = g[s[1]] + get_direct_distance(s[1], s_p[0])
            parent[s_p[0]] = s[1]
            if fringe_has_position(fringe, s_p[0]):  # need to fix open list stuff
                fringe_remove(fringe, s_p[0])
            heapq.heappush(fringe, (g[s_p[0]] + h(s_p[0], goal), s_p[0]))


def is_blocked(blocklist, position):
    if position in blocklist:
        return True
    else:
        return False


# s and sprime are tuples of (x,y)
# MIGHT NEED TO IMPLEMENT GRID
def line_of_sight(s, sprime, blocklist):
    x0 = s[0]
    y0 = s[1]
    x1 = sprime[0]
    y1 = sprime[1]
    f = 0
    dy = y1 - y0
    dx = x1 - x0
    if dy < 0:
        dy = -dy
        sy = -1
    else:
        sy = 1
    if dx < 0:
        dx = -dx
        sx = -1
    else:
        sx = 1
    if dx >= dy:
        while x0 != x1:
            f = f + dy
            if f >= dx:
                if is_blocked(blocklist, (x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2))):
                    return False
                y0 = y0 + sy
                f = f - dx
            if f != 0 and is_blocked(blocklist, (x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2))):
                return False
            if dy == 0 and is_blocked(blocklist, (x0 + ((sx - 1) / 2), y0)) \
                    and is_blocked(blocklist, (x0 + ((sx - 1) / 2), y0 - 1)):
                return False
            x0 = x0 + sx
    else:
        while y0 != y1:
            f = f + dx
            if f >= dy:
                if is_blocked(blocklist, (x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2))):
                    return False
                x0 = x0 + sx
                f = f - dy
            if f != 0 and is_blocked(blocklist, (x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2))):
                return False
            if dx == 0 and is_blocked(blocklist, (x0, y0 + ((sy - 1) / 2))) \
                    and is_blocked(blocklist, (x0 - 1, (sy - 1) / 2)):
                return False
            y0 = y0 + sy

    return True


def fringe_has_position(fringe, position):
    for p in fringe:
        if p[1] == position:
            return True
    return False


def fringe_remove(fringe, position):
    for p in fringe:
        if p[1] == position:
            fringe.remove(p)
            break

# position represented as (x, y)
def point_not_OFR(position, x_lim, y_lim):
    if (position[0] > x_lim+1 or position[0] < 1 or position[1] > y_lim+1 or position[1] < 1):
        return False
    return True


def succ(position, blocklist, x_lim, y_lim):
    '''
    a child should be:
     1: near the parent (8 dots)
     2: not out of range
     3: not crossing the blocked cell
    a child is represented as ((x,y),c), where c represents the direct distance between x,y
    '''
    childlist = []

    top_left = (position[0] - 1, position[1] - 1)
    if (top_left not in blocklist) and point_not_OFR(top_left, x_lim, y_lim):
        childlist.append((top_left, math.sqrt(2)))

    top = (position[0], position[1] - 1)
    if (not ((top_left in blocklist) and (top in blocklist))) and point_not_OFR(top, x_lim, y_lim):
        childlist.append((top, 1))

    top_right = (position[0] + 1, position[1] - 1)
    if (top not in blocklist) and point_not_OFR(top_right, x_lim, y_lim):
        childlist.append((top_right, math.sqrt(2)))

    right = (position[0] + 1, position[1])
    if (not ((position in blocklist) and (top in blocklist))) and point_not_OFR(right, x_lim, y_lim):
        childlist.append((right, 1))

    bottom_right = (position[0] + 1, position[1] + 1)
    if (position not in blocklist) and point_not_OFR(bottom_right, x_lim, y_lim):
        childlist.append((bottom_right, math.sqrt(2)))

    left = (position[0] - 1, position[1])
    if (not ((left in blocklist) and (top_left in blocklist))) and point_not_OFR(left, x_lim, y_lim):
        childlist.append((left, 1))

    bottom_left = (position[0] - 1, position[1] + 1)
    if (left not in blocklist) and point_not_OFR(bottom_left, x_lim, y_lim):
        childlist.append((bottom_left, math.sqrt(2)))

    bottom = (position[0], position[1] + 1)
    if (not ((left in blocklist) and (position in blocklist))) and point_not_OFR(bottom, x_lim, y_lim):
        childlist.append((bottom, 1))

    return childlist


def plot_grid(start, goal, blocklist, x_lim, y_lim, x_path, y_path):
    x = range(1, x_lim + 2)
    y = range(1, y_lim + 2)
    fig, ax = plt.subplots(1, 1, figsize=(30, 15), dpi=50)
    ax.set_xlim((1, x_lim + 1))
    ax.set_ylim((1, y_lim + 1))
    ax.invert_yaxis()
    ax.xaxis.tick_top()
    ax.set_xticks(x)
    ax.set_yticks(y)
    ax.grid(True)
    for block in blocklist:
        ax.add_patch(
            patches.Rectangle(
                block,
                1,
                1,
                edgecolor='black',
                facecolor='gray',
                fill=True
            ))
    ax.plot(start[0], start[1], marker="o", markersize=15, markeredgecolor="green", markerfacecolor="green")
    ax.plot(goal[0], goal[1], marker="o", markersize=15, markeredgecolor="red", markerfacecolor="red")
    ax.plot(x_path, y_path, color="blue")
    ax.set_title('Grid',fontdict={'fontsize': 48})
    plt.show()

# generate multiple grids, save them to a file folder
def generate_mult_grid(x_lim, y_lim, grid_num):
    x = range(1, x_lim + 2)
    y = range(1, y_lim + 2)
    result_list = []
    goal_list = []
    g_list = []
    result_list2 = []
    g_list2 = []
    for i in range(grid_num):
        blocklist = generate_blocklist(x_lim,y_lim)
        start, goal = generate_start_and_goal(x_lim, y_lim)
        result, parent, g = aStar(start, goal, blocklist, x_lim, y_lim)
        result_list.append(result)
        goal_list.append(goal)
        g_list.append(g)
        if result == "Path Found":
            pathlist, x_path, y_path = generate_pathlist(parent, start, goal)
        else:
            pathlist, x_path, y_path = []

        result2, parent2, g2 = thetaStar(start, goal, blocklist, x_lim, y_lim)
        result_list2.append(result2)
        g_list2.append(g2)
        if result2 == "Path Found":
            pathlist2, x_path2, y_path2 = generate_pathlist(parent2, start, goal)
        else:
            pathlist2, x_path2, y_path2 = []

        fig, ax = plt.subplots(1, 1, figsize=(30, 15), dpi=50)
        ax.set_xlim((1, x_lim + 1))
        ax.set_ylim((1, y_lim + 1))
        ax.invert_yaxis()
        ax.xaxis.tick_top()
        ax.set_xticks(x)
        ax.set_yticks(y)
        ax.grid(True)
        for block in blocklist:
            ax.add_patch(
                patches.Rectangle(
                    block,
                    1,
                    1,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                ))
        ax.plot(start[0], start[1], marker="o", markersize=15, markeredgecolor="green", markerfacecolor="green")
        ax.plot(goal[0], goal[1], marker="o", markersize=15, markeredgecolor="red", markerfacecolor="red")
        if result == "Path Found":
            ax.plot(x_path, y_path, color="blue")
        if result2 == "Path Found":
            ax.plot(x_path2, y_path2, color="purple")
        ax.set_title('Grid', fontdict={'fontsize': 48})
        label_text = 'Grid ' + str(i) + ': ' + result
        file_name = './grids/Grid_' + str(i) + '.png'
        ax.set_title(label_text, fontdict={'fontsize': 48})
        plt.savefig(file_name,format='png')
        # also save the grid by txt files in gridFiles
        datalist = generate_grid_datalist(start, goal, x_lim, y_lim, blocklist)
        gridFile_name = './gridFiles/gridFile_' + str(i) + '.txt'
        gridFile = open(gridFile_name, "w+")
        gridFile.writelines(datalist)
        gridFile.close()
    return result_list, goal_list, g_list, result_list2, g_list2

def generate_grid_datalist(start, goal, x_lim, y_lim, blocklist):
    datalist = []
    datalist.append(tuple_to_string2(start))
    datalist.append(tuple_to_string2(goal))
    datalist.append(tuple_to_string2((x_lim, y_lim)))
    for i in range(1, x_lim+1):
        for j in range(1, y_lim + 1):
            if (i,j) in blocklist:
                datalist.append(tuple_to_string3((i, j, 1)))
            else:
                datalist.append(tuple_to_string3((i, j, 0)))
    return datalist

def tuple_to_string2(tuple):
    tuple_s = str(tuple[0]) + ' ' + str(tuple[1]) + '\n'
    return tuple_s

def tuple_to_string3(tuple):
    tuple_s = str(tuple[0]) + ' ' + str(tuple[1]) + ' ' + str(tuple[2]) + '\n'
    return tuple_s

def generate_pathlist(parent, start, goal):
    pathlist = [goal]
    a = goal
    while a != start:
        a = parent[a]
        pathlist.append(a)
    x_path = [a[0] for a in pathlist]
    y_path = [a[1] for a in pathlist]
    return pathlist, x_path, y_path


def find_path_num(result_list):
    count = 0
    for i in result_list:
        if i == 'Path Found':
            count += 1
    return count


def load_multi_grids():
    result_list = []
    goal_list = []
    g_list = []
    result_list2 = []
    g_list2 = []
    grid_num = 0
    for filename in os.scandir('./gridFiles'):
        if filename.is_file():
            gridFile = open(filename, "r")
            blocklist = []
            i = 0
            for line in gridFile:
                line_list = line.split(" ")
                if i == 0:
                    start = (int(line_list[0]),int(line_list[1]))
                elif i == 1:
                    goal = (int(line_list[0]),int(line_list[1]))
                elif i == 2:
                    x_lim = int(line_list[0])
                    y_lim = int(line_list[1])
                else:
                    if int(line_list[2]) == 1:
                        blocklist.append((int(line_list[0]),int(line_list[1])))
                i += 1

            result, parent, g = aStar(start, goal, blocklist, x_lim, y_lim)
            result_list.append(result)
            goal_list.append(goal)
            g_list.append(g)
            if result == "Path Found":
                pathlist, x_path, y_path = generate_pathlist(parent, start, goal)
            else:
                pathlist, x_path, y_path = []

            result2, parent2, g2 = thetaStar(start, goal, blocklist, x_lim, y_lim)
            result_list2.append(result2)
            g_list2.append(g2)
            if result == "Path Found":
                pathlist2, x_path2, y_path2 = generate_pathlist(parent2, start, goal)
            else:
                pathlist2, x_path2, y_path2 = []

            x = range(1, x_lim + 2)
            y = range(1, y_lim + 2)
            fig, ax = plt.subplots(1, 1, figsize=(30, 15), dpi=50)
            ax.set_xlim((1, x_lim + 1))
            ax.set_ylim((1, y_lim + 1))
            ax.invert_yaxis()
            ax.xaxis.tick_top()
            ax.set_xticks(x)
            ax.set_yticks(y)
            ax.grid(True)
            for block in blocklist:
                ax.add_patch(
                    patches.Rectangle(
                        block,
                        1,
                        1,
                        edgecolor='black',
                        facecolor='gray',
                        fill=True
                    ))
            ax.plot(start[0], start[1], marker="o", markersize=15, markeredgecolor="green", markerfacecolor="green")
            ax.plot(goal[0], goal[1], marker="o", markersize=15, markeredgecolor="red", markerfacecolor="red")
            if result == "Path Found":
                ax.plot(x_path, y_path, color="blue")
            if result2 == "Path Found":
                ax.plot(x_path2, y_path2, color="purple")
            ax.set_title('Grid', fontdict={'fontsize': 48})
            label_text = 'Grid ' + str(grid_num) + ': ' + result
            file_name = './grids/Grid_' + str(grid_num) + '.png'
            ax.set_title(label_text, fontdict={'fontsize': 48})
            plt.savefig(file_name, format='png')
            grid_num += 1
    return result_list, goal_list, g_list, grid_num, result_list2, g_list2

if __name__ == '__main__':
    # x_lim and y_lim is the size of the grid
    window = tk.Tk()
    window.title('Path Search')
    window.geometry('900x600')
    l = tk.Label(window, text='Click one option to continue', bg='green', font=('Arial', 20), width=30, height=1)
    l.pack()
    t = tk.Text(window, height=10, width=100)
    t.insert('insert', "Left button to generate random grid and Right button to load grid data.\n"
                       "In the grid, the green point represents the position of start point.\n"
                       "The red point represents the position of goal point.\n"
                       "The blue line represents the path searched by A* algorithm.\n"
                       "The purple line represents the path searched by Theta* algorithm.\n"
                       "If you want to load data, you must put all the grid files under a folder named 'gridFiles'"
                       "in the directory which has this main.py\n"
                       "Please manually delete the generated grids and files if you want to test more than one time, "
                       "but do not remove the folders")
    t.pack()
    tk.Label(window, text='width of grid (x):', font=('Arial', 14)).place(x=30, y=200)
    tk.Label(window, text='height of grid (y):', font=('Arial', 14)).place(x=30, y=240)
    tk.Label(window, text='number of grid:', font=('Arial', 14)).place(x=30, y=280)
    # input for generating random grids
    x_lim_s = tk.StringVar()
    x_lim_s.set('100')
    entry_x_lim = tk.Entry(window, textvariable=x_lim_s, font=('Arial', 14))
    entry_x_lim.place(x=200, y=205)
    y_lim_s = tk.StringVar()
    y_lim_s.set('50')
    entry_y_lim = tk.Entry(window, textvariable=y_lim_s, font=('Arial', 14))
    entry_y_lim.place(x=200, y=245)
    grid_num_s = tk.StringVar()
    grid_num_s.set('50')
    grid_num_s = tk.Entry(window, textvariable=grid_num_s, font=('Arial', 14))
    grid_num_s.place(x=200, y=285)
    def generate_rand_grids_func():
        x_lim = int(x_lim_s.get())
        y_lim = int(y_lim_s.get())
        grid_num = int(grid_num_s.get())
        result_list, goal_list, g_list, result_list2, g_list2 = generate_mult_grid(x_lim,y_lim, grid_num)
        has_path_num = find_path_num(result_list)
        has_path_num2 = find_path_num(result_list2)
        # cur_grid_num control which grid to show
        cur_grid_num = tk.IntVar()
        # generate new window to show grids
        window_grid = tk.Toplevel(window)
        window_grid.geometry('1600x900')
        window_grid.title('grid window')
        canvas = tk.Canvas(window_grid, bg='white', height=750, width=1500)
        canvas.pack()
        img_file_list = []
        for i in range(grid_num):
            img_file_list.append(tk.PhotoImage(file='./grids/Grid_' + str(i) + '.png'))
        #image_file = tk.PhotoImage(file='./grids/Grid_' + str(cur_grid_num.get()) + '.png')
        image = canvas.create_image(750, 0, anchor='n', image=img_file_list[0])
        def go_prev_grid():
            if cur_grid_num.get() > 0:
                cur_grid_num.set(cur_grid_num.get()-1)
                canvas.itemconfig(image, image=img_file_list[cur_grid_num.get()])
        btn_prev = tk.Button(window_grid, text='prev', command=go_prev_grid)
        btn_prev.place(x=20, y=850)

        def go_next_grid():
            if cur_grid_num.get() < grid_num - 1:
                cur_grid_num.set(cur_grid_num.get()+1)
                canvas.itemconfig(image, image=img_file_list[cur_grid_num.get()])
        btn_next = tk.Button(window_grid, text='next', command=go_next_grid)
        btn_next.place(x=60, y=850)

        tk.Label(window_grid, text='this choose which \n image to display', font=('Arial', 10)).place(x=20, y=800)
        tk.Label(window_grid, text='enter a position to get h, g, f \n for current grid:', font=('Arial', 10)).place(x=300, y=800)
        tk.Label(window_grid, text='x: ', font=('Arial', 14)).place(x=280, y=850)
        tk.Label(window_grid, text='y: ', font=('Arial', 14)).place(x=380, y=850)
        target_x_s = tk.StringVar()
        entry_target_x = tk.Entry(window_grid, textvariable=target_x_s, font=('Arial', 14), width=5)
        entry_target_x.place(x=300, y=850)
        target_y_s = tk.StringVar()
        entry_target_y = tk.Entry(window_grid, textvariable=target_y_s, font=('Arial', 14), width=5)
        entry_target_y.place(x=400, y=850)
        hgf = tk.Text(window_grid, height=6, width=100)
        hgf.place(x=550, y=800)
        def get_hgf():
            target_x = int(target_x_s.get())
            target_y = int(target_y_s.get())
            hgf.delete("1.0", "end")
            target_h = h((target_x, target_y), goal_list[cur_grid_num.get()])
            hgf.insert('end', "the h value is: " + str(target_h) + '\n')
            if (target_x, target_y) in g_list[cur_grid_num.get()]:
                target_g = g_list[cur_grid_num.get()][(target_x, target_y)]
                hgf.insert('end', "the g value for A* is: " + str(target_g) + '\n')
                hgf.insert('end', "the f value for A* is: " + str(target_h + target_g) + '\n')
            else:
                hgf.insert('end', "the g and f value for A* does not exist\n")
            if (target_x, target_y) in g_list2[cur_grid_num.get()]:
                target_g2 = g_list2[cur_grid_num.get()][(target_x, target_y)]
                hgf.insert('end', "the g value for Theta* is: " + str(target_g2) + '\n')
                hgf.insert('end', "the f value for Theta* is: " + str(target_h + target_g2) + '\n')
            else:
                hgf.insert('end', "the g and f value for theta* does not exist\n")

        btn_conf = tk.Button(window_grid, text='confirm', command=get_hgf)
        btn_conf.place(x=480, y=850)
        tk.Label(window_grid, text='there are total ' + str(has_path_num) + ' \n' +
                                   'has a path for A*,\ntotal ' + str(has_path_num2) + ' \n' +
                                   'has a path for Theta*\n' +
                                   'out of ' + str(grid_num) + ' grids',
                 font=('Arial', 12)).place(x=1400, y=800)
        window_grid.mainloop()

    btn_generate_rand = tk.Button(window, text='generate random grids', command=generate_rand_grids_func)
    btn_generate_rand.place(x=30, y=320)

    def load_grids_func():
        result_list, goal_list, g_list, grid_num, result_list2, g_list2 = load_multi_grids()
        has_path_num = find_path_num(result_list)
        has_path_num2 = find_path_num(result_list2)
        # cur_grid_num control which grid to show
        cur_grid_num = tk.IntVar()
        # generate new window to show grids
        window_grid = tk.Toplevel(window)
        window_grid.geometry('1600x900')
        window_grid.title('grid window')
        canvas = tk.Canvas(window_grid, bg='white', height=750, width=1500)
        canvas.pack()
        img_file_list = []
        for i in range(grid_num):
            img_file_list.append(tk.PhotoImage(file='./grids/Grid_' + str(i) + '.png'))
        # image_file = tk.PhotoImage(file='./grids/Grid_' + str(cur_grid_num.get()) + '.png')
        image = canvas.create_image(750, 0, anchor='n', image=img_file_list[0])

        def go_prev_grid():
            if cur_grid_num.get() > 0:
                cur_grid_num.set(cur_grid_num.get() - 1)
                canvas.itemconfig(image, image=img_file_list[cur_grid_num.get()])

        btn_prev = tk.Button(window_grid, text='prev', command=go_prev_grid)
        btn_prev.place(x=20, y=850)

        def go_next_grid():
            if cur_grid_num.get() < grid_num - 1:
                cur_grid_num.set(cur_grid_num.get() + 1)
                canvas.itemconfig(image, image=img_file_list[cur_grid_num.get()])

        btn_next = tk.Button(window_grid, text='next', command=go_next_grid)
        btn_next.place(x=60, y=850)

        tk.Label(window_grid, text='this choose which \n image to display', font=('Arial', 10)).place(x=20, y=800)
        tk.Label(window_grid, text='enter a position to get h, g, f \n for current grid:', font=('Arial', 10)).place(
            x=300, y=800)
        tk.Label(window_grid, text='x: ', font=('Arial', 14)).place(x=280, y=850)
        tk.Label(window_grid, text='y: ', font=('Arial', 14)).place(x=380, y=850)
        target_x_s = tk.StringVar()
        entry_target_x = tk.Entry(window_grid, textvariable=target_x_s, font=('Arial', 14), width=5)
        entry_target_x.place(x=300, y=850)
        target_y_s = tk.StringVar()
        entry_target_y = tk.Entry(window_grid, textvariable=target_y_s, font=('Arial', 14), width=5)
        entry_target_y.place(x=400, y=850)
        hgf = tk.Text(window_grid, height=6, width=100)
        hgf.place(x=550, y=800)

        def get_hgf():
            target_x = int(target_x_s.get())
            target_y = int(target_y_s.get())
            hgf.delete("1.0", "end")
            target_h = h((target_x, target_y), goal_list[cur_grid_num.get()])
            hgf.insert('end', "the h value is: " + str(target_h) + '\n')
            if (target_x, target_y) in g_list[cur_grid_num.get()]:
                target_g = g_list[cur_grid_num.get()][(target_x, target_y)]
                hgf.insert('end', "the g value for A* is: " + str(target_g) + '\n')
                hgf.insert('end', "the f value for A* is: " + str(target_h + target_g) + '\n')
            else:
                hgf.insert('end', "the g and f value for A* does not exist\n")
            if (target_x, target_y) in g_list2[cur_grid_num.get()]:
                target_g2 = g_list2[cur_grid_num.get()][(target_x, target_y)]
                hgf.insert('end', "the g value for Theta* is: " + str(target_g2) + '\n')
                hgf.insert('end', "the f value for Theta* is: " + str(target_h + target_g2) + '\n')
            else:
                hgf.insert('end', "the g and f value for theta* does not exist\n")

        btn_conf = tk.Button(window_grid, text='confirm', command=get_hgf)
        btn_conf.place(x=480, y=850)
        tk.Label(window_grid, text='there are total ' + str(has_path_num) + ' \n' +
                   'has a path for A*,\ntotal ' + str(has_path_num2) + ' \n' +
                    'has a path for Theta*\n' +
                    'out of ' + str(grid_num) + ' grids',
                 font=('Arial', 12)).place(x=1400, y=800)
        window_grid.mainloop()

    btn_load_grid = tk.Button(window, text='load from files', command=load_grids_func)
    btn_load_grid.place(x=500, y=320)

    window.mainloop()
