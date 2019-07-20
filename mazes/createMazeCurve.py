import numpy as np
import cv2

# (height, width, channel) of the image
shape = (50,50,1)
#maze = np.ones(shape)
# maze = np.full(shape, 255)
maze = np.zeros(shape) # this is fully black => blocked

freeShape = (10,10,1)
free = np.full(freeShape, 255)
#obstacle = np.zeros(obstacleShape)

m = [[0, 10],[10, 10],[20, 10],[30, 10],[ 30, 20],[20, 20],[20, 30],[20, 40],[10, 40]]

 # row , col
for i in m:
	r = i[0]
	c = i[1]
	maze[r:r+free.shape[0], c:c+free.shape[1],:] = free

# # assigns the part of the maze image as obstracle 
# maze[r1:r1+obstacle.shape[0], c1:c1+obstacle.shape[1],:] = obstacle
# maze[r2:r2+obstacle.shape[0], c2:c2+obstacle.shape[1],:] = obstacle



# # r3, c3 = 25, 125
# # # r4, c4 = 125, 125
# # maze[r3:r3+obstacle.shape[0], c3:c3+obstacle.shape[1],:] = obstacle
# # maze[r4:r4+obstacle.shape[0], c4:c4+obstacle.shape[1],:] = obstacle
# # Mat b = cv.fromarray(maze)

# obstacleShapeTwo = (125,15,1)
# obstacleShapeTwo = np.zeros(obstacleShapeTwo)

# r2, c2 = 25, 100
# r3, c3 = 50, 140
# r4, c4 = 25, 180
# maze[r2:r2+obstacleShapeTwo.shape[0], c2:c2+obstacleShapeTwo.shape[1],:] = obstacleShapeTwo
# maze[r3:r3+obstacleShapeTwo.shape[0], c3:c3+obstacleShapeTwo.shape[1],:] = obstacleShapeTwo
# maze[r4:r4+obstacleShapeTwo.shape[0], c4:c4+obstacleShapeTwo.shape[1],:] = obstacleShapeTwo

cv2.imwrite('maze_2.png', maze)
# cv2.imshow('maze_basic', maze)
cv2.waitKey(0)
cv2.destroyAllWindows()