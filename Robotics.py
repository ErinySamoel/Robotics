#!/usr/bin/env python
# coding: utf-8

# In[53]:




import numpy as np

import heapq

import matplotlib.pyplot as plt

from matplotlib.pyplot import figure



# In[69]:


from numpy.core.fromnumeric import shape
grid=np.zeros((1000,1000), dtype=int)
print(grid.shape)
for i in range(0,200):
    for j in range(0,200):
        grid[i][j]=1

for i in range(400,600):
    for j in range(200,1000):
        grid[j][i]=1

for i in range(700,800):
    for j in range(0,600):
        grid[j][i]=1

start = (950,100)

goal = (250,950)


# In[74]:


fig, ax = plt.subplots(figsize=(12,12))
fig = plt.figure()

ax.imshow(grid, cmap=plt.cm.Dark2)

ax.scatter(start[1],start[0], s = 100)#, marker = ".", color = "yellow"

ax.scatter(goal[1],goal[0], s = 100)# marker = ".", color = "red",

plt.show()


# In[75]:


def manhattanHeuristic( newNode,  end):
        return (abs(newNode[0] - end[0]) + abs(newNode[1]- end[1]));
    


# In[ ]:


def astar(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    close_set = set()

    came_from = {}

    gscore = {start:0}

    fscore = {start:manhattanHeuristic(start, goal)}

    oheap = []

    heapq.heappush(oheap, (fscore[start], start))
 

    while oheap:

        current = heapq.heappop(oheap)[1]

        if current == goal:

            data = []

            while current in came_from:

                data.append(current)

                current = came_from[current]

            return data

        close_set.add(current)

        for i, j in neighbors:

            neighbor = current[0] + i, current[1] + j

            tentative_g_score = gscore[current] + manhattanHeuristic(current, neighbor)

            if 0 <= neighbor[0] < array.shape[0]:

                if 0 <= neighbor[1] < array.shape[1]:                

                    if array[neighbor[0]][neighbor[1]] == 1:

                        continue

                else:

                    # array bound y walls

                    continue

            else:

                # array bound x walls

                continue
 

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):

                continue
 

            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:

                came_from[neighbor] = current

                gscore[neighbor] = tentative_g_score

                fscore[neighbor] = tentative_g_score + manhattanHeuristic(neighbor, goal)

                heapq.heappush(oheap, (fscore[neighbor], neighbor))
 

    return False

route = astar(grid, start, goal)

route = route + [start]

route = route[::-1]

print(route)


# In[63]:



x_coords = []

y_coords = []

for i in (range(0,10)):#len(route)

    x = route[i][0]

    y = route[i][1]

    x_coords.append(x)

    y_coords.append(y)

# plot map and path

fig, ax = plt.subplots(figsize=(12,12))

ax.imshow(grid, cmap=plt.cm.Dark2)

ax.scatter(start[1],start[0], marker = ".", color = "yellow", s = 1000)

ax.scatter(goal[1],goal[0], marker =  "." ,color = "red", s = 1000)

ax.plot(y_coords,x_coords, color = "black")

plt.show()


# In[64]:


for i in (range(0,len(route))):#len(route)

    x = route[i][0]

    y = route[i][1]

    x_coords.append(x)

    y_coords.append(y)

# plot map and path

fig, ax = plt.subplots(figsize=(12,12))

ax.imshow(grid, cmap=plt.cm.Dark2)

ax.scatter(start[1],start[0], marker = ".", color = "yellow", s = 1000)

ax.scatter(goal[1],goal[0], marker =  "." ,color = "red", s = 1000)

ax.plot(y_coords,x_coords, color = "black")

plt.show()


# In[ ]:




