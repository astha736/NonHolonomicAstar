import numpy as np
import cv2

# (height, width, channel) of the image
shape = (100,200,1)
#maze = np.ones(shape)
maze = np.full(shape, 255)

# obstacleShape = (50,50,1)
# obstacle = np.zeros(obstacleShape)
obstacleShape = (75,75,1)
obstacle = np.zeros(obstacleShape)
#obstacle = np.zeros(obstacleShape)

# r1, c1 = 25, 25
# r2, c2 = 125, 25 # row , col
# r3, c3 = 25, 125
# r4, c4 = 125, 125



r1, c1 = 10, 10
# r2, c2 = 115, 10 # row , col
r3, c3 = 10, 115
# r4, c4 = 115, 115


# assigns the part of the maze image as obstracle 
maze[r1:r1+obstacle.shape[0], c1:c1+obstacle.shape[1],:] = obstacle
# maze[r2:r2+obstacle.shape[0], c2:c2+obstacle.shape[1],:] = obstacle
maze[r3:r3+obstacle.shape[0], c3:c3+obstacle.shape[1],:] = obstacle
# maze[r4:r4+obstacle.shape[0], c4:c4+obstacle.shape[1],:] = obstacle
# Mat b = cv.fromarray(maze)

cv2.imwrite('maze_basic_7.png', maze)
# cv2.imshow('maze_basic', maze)
cv2.waitKey(0)
cv2.destroyAllWindows()