
grid = [[0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0]]

heuristic_Astar = [ [9, 8, 7, 6, 5, 4],
                    [8, 7, 6, 5, 4, 3],
                    [7, 6, 5, 4, 3, 2],
                    [6, 5, 4, 3, 2, 1],
                    [5, 4, 3, 2, 1, 0]]

heuristic_normal_search = [ [0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']


def search(grid,init,goal,cost,heuristic):

    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expand = [[-1 for row in range(len(grid[0]))] for col in range(len(grid[1]))]


    x = init[0]
    y = init[1]
    g = 0
    h = heuristic[0][1]
    f = g + h

    open = [[f, g, h, x, y]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0

    while not found and not resign:

        if len(open) == 0:
            resign = True
            return 'fail'
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            x = next[3]
            y = next[4]
            g = next[1]
            expand[x][y] = count
            count +=1
            
            if x == goal[0] and y == goal[1]:
                found = True
                print(next,"\n")
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            h2 = heuristic[x2][y2]
                            f2 = g2 + h2
                            open.append([f2, g2, h2, x2, y2])
                            closed[x2][y2] = 1

    return expand

# A* search with A* heuristic function
expand = search(grid,init,goal,cost,heuristic=heuristic_Astar)

for i in range(len(expand)):
    print(expand[i])

'''
# normal search with neglecting heuristic function term with zeros
expand = search(grid,init,goal,cost,heuristic=heuristic_normal_search)

for i in range(len(expand)):
    print(expand[i])
'''
