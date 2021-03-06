import numpy as np
import cv2

# (height, width, channel) of the image
shape = (200,200,1)
#maze = np.ones(shape)
maze = np.full(shape, 255)

obstacleShapeOne = (150,100,1)
obstacleShapeOne = np.zeros(obstacleShapeOne)
#obstacle = np.zeros(obstacleShape)

r1, c1 = 25, 25

# assigns the part of the maze image as obstracle 
maze[r1:r1+obstacleShapeOne.shape[0], c1:c1+obstacleShapeOne.shape[1],:] = obstacleShapeOne

obstacleShapeTwo = (50,50,1)
obstacleShapeTwo = np.zeros(obstacleShapeTwo)

r2, c2 = 125, 125
maze[r2:r2+obstacleShapeTwo.shape[0], c2:c2+obstacleShapeTwo.shape[1],:] = obstacleShapeTwo

# Mat b = cv.fromarray(maze)

cv2.imwrite('maze_basic_1.png', maze)
# cv2.imshow('maze_basic', maze)
cv2.waitKey(0)
cv2.destroyAllWindows()
