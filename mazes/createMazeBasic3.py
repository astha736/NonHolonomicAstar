import numpy as np
import cv2

# (height, width, channel) of the image
shape = (200,200,1)
#maze = np.ones(shape)
maze = np.full(shape, 255)

obstacleShape = (15,15,1)
obstacle = np.zeros(obstacleShape)
#obstacle = np.zeros(obstacleShape)

r1, c1 = 10, 10
r2, c2 = 10, 30

# assigns the part of the maze image as obstracle 
maze[r1:r1+obstacle.shape[0], c1:c1+obstacle.shape[1],:] = obstacle
maze[r2:r2+obstacle.shape[0], c2:c2+obstacle.shape[1],:] = obstacle
# Mat b = cv.fromarray(maze)

cv2.imwrite('maze_basic_3.png', maze)
# cv2.imshow('maze_basic', maze)
cv2.waitKey(0)
cv2.destroyAllWindows()