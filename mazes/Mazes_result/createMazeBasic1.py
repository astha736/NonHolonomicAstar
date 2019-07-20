import numpy as np
import cv2

# (height, width, channel) of the image
shape = (200,200,1)
#maze = np.ones(shape)
maze = np.full(shape, 255)

obstacleShape = (150,100,1)
obstacle = np.zeros(obstacleShape)
#obstacle = np.zeros(obstacleShape)

r, c = 25, 25

# assigns the part of the maze image as obstracle 
maze[r:r+obstacle.shape[0], c:c+obstacle.shape[1],:] = obstacle

# Mat b = cv.fromarray(maze)

cv2.imwrite('maze_basic_1.png', maze)
# cv2.imshow('maze_basic', maze)
cv2.waitKey(0)
cv2.destroyAllWindows()
