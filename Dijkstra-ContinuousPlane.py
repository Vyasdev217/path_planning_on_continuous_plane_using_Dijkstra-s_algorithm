# Dijkstra's algorithm for path planning on continuous space
# Copyright (C) 2023 Vyasdev
# This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.
# This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
# You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

"""
Dijkstra's algorithm for path planning on continuous space

Input: Map image
    Starting point(Only 1 pixel): Blue(0,0,255)
    Destination point(Only 1 pixel): Green(0,255,0)
    Obstacles: Black(0,0,0)
    Hidden obstacles[Not to be shown in output. Mainly used for defining domain in the map]: (100,100,100)

Output: Map image with path
    Path: Red(255,0,0)
"""

import numpy as np
from PIL import Image, ImageOps
from sys import exit
import pygame


mappath=input("Path of map: ")
show_process=True # To view the algorithm in pygame

try:
    # Load the map
    groundpng=Image.open(mappath)
except:
    print("Failed to load the map")
    exit(1)
'''
    __Map color definition__(R,G,B)
    Start=(0,0,255)
    Dest=(0,255,0)
    Obstacle=(0,0,0)
'''

# Expand the map with obastacle border
groundpng=ImageOps.expand(groundpng, border=(1,1,1,1), fill=(0,0,0))

# Initialize the current location list
current_locs=[]

# Initialize the previous location list for traceback
prevloc=[[0 for i in range(groundpng.size[1])] for j in range(groundpng.size[0])]

# Initialize an array to store the map data based on pixel color
ground_data=np.zeros((groundpng.size[0],groundpng.size[1]),float)
'''
    __value definiton__
    Destination = 1
    Default_level = 0
    Start = -1
    Obstacle = -2
'''
start_count=0
destination_count=0
# Load the map data to the array
for i in range(groundpng.size[0]):

    for j in range(groundpng.size[1]):
        
        # Exit if there is multiple start or destination point
        if start_count>1 or destination_count>1:
            print("Invalid input file: Multiple start or destination point detected")
            exit(1)

        if groundpng.getpixel((i,j))[:3]==(0,0,0):
            ground_data[i,j]=-2

        elif groundpng.getpixel((i,j))[:3]==(0,255,0):
            ground_data[i,j]=1
            destination_count+=1
            destination=(i,j) # Note the destination location

        elif groundpng.getpixel((i,j))[:3]==(0,0,255):
            ground_data[i,j]=-1
            start_count+=1
            start=(i,j) # Note the start location

        else:
            # Hidden obstacles
            #if groundpng.getpixel((i,j))[:3]==(100,100,100):
            #    ground_data[i,j]=-2
            
            # If the pixel is not an obstacle, start or destination, color it white (This hides the hidden obstacles from the output)
            groundpng.paste(Image.new(mode="RGB",size=(1,1),color=(255,255,255)),(i,j))
            
            # Expand the non-hidden obstacles in map array to avoid collition but not in the map image
            for k in range(-3,4):
                for l in range(-3,4):
                    if i+k<=0 or j+l<=0 or i+k>=groundpng.size[0]-1 or j+l>=groundpng.size[1]-1:
                        continue
                    if groundpng.getpixel((i+k,j+l))[:3]==(0,0,0):
                        ground_data[i,j]=-2

# Expand the start and destination only in map image for better visibility
for i in range(start[0]-3,start[0]+4):
    for j in range(start[1]-3,start[1]+4):
        groundpng.paste(Image.new(mode="RGB",size=(1,1),color=(0,0,255)),(i,j))
for i in range(destination[0]-3,destination[0]+4):
    for j in range(destination[1]-3,destination[1]+4):
        groundpng.paste(Image.new(mode="RGB",size=(1,1),color=(0,255,0)),(i,j))

# Save the map image as temporary file to load in pygame
groundpng.save("temp.png")

# Get the map size
IMG_SIZE=groundpng.size

if show_process:
    # Initialize pygame window
    pygame.init()
    DISPLAYSURF = pygame.display.set_mode(IMG_SIZE)
    pygame.display.set_caption("Path planning algorithm")
    background = pygame.image.load("temp.png").convert()
    background_rect = background.get_rect()
    DISPLAYSURF.blit(background, background_rect)
    pygame.display.flip()

# Append starting point in current location list
current_locs.append(start)
prevloc[start[0]][start[1]]=start

# Function to get the next points from the current point and update previous location list for traceback
def nextPoints(p):
    ret=[]
    # I think this part can be improved
    for i,j in [(1,0),(0,1),(-1,0),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
        if ground_data[p[0]+i,p[1]+j]>=0 and prevloc[p[0]+i][p[1]+j]==0:
            ret.append((p[0]+i,p[1]+j))
            prevloc[p[0]+i][p[1]+j]=p
    '''
    for i in range(1,-2,-1):
        for j in range(1,-2,-1):
            if (i==0 and j==0):
                continue
            if ground_data[p[0]+i,p[1]+j]>=0 and prevloc[p[0]+i][p[1]+j]==0:
                ret.append((p[0]+i,p[1]+j))
                prevloc[p[0]+i][p[1]+j]=p
    '''
    return ret

while True:
    # Fetch a node from currect locations list
    p=current_locs[0]

    # Break if destination is found
    if ground_data[p[0],p[1]]==1:
        break

    # Append unvisited adjacent nodes of selected node to current location list and remove the fetched node from it
    current_locs.extend(nextPoints(p))
    current_locs=current_locs[1:]

    # Show the process in pygame
    if show_process:
        pygame.event.get()
        pygame.draw.circle(DISPLAYSURF, (255,0,0), tuple(p),1,0)
        pygame.display.flip()

# Show the path in pygame(Initialize pygame window)
if show_process:
    DISPLAYSURF.blit(background, background_rect)

# Track back the path from destination to starting point using the sata stored un prevloc list and plot on map
p=current_locs[0]
while p!=start:
    #groundpng.putpixel(p,(255,0,0))
    for i in range(p[0]-1,p[0]+2):
        for j in range(p[1]-1,p[1]+2):
            groundpng.putpixel((i,j),(255,0,0))
            if show_process:
                pygame.draw.circle(DISPLAYSURF, (255,0,0), (i,j),1,0)
                pygame.display.flip()
    p=prevloc[p[0]][p[1]]

if show_process:
    from time import sleep
    pygame.quit()

# Expand the map with white border
groundpng=ImageOps.expand(groundpng, border=(15,15,15,15), fill=(255,255,255))

# Save the map with output
groundpng.save("result.png")

groundpng.close()
