#!/bin/python3

import cv2
import numpy as np

# Define the cell size (in pixels)
CELL_SIZE = 10  # Example cell size

# Load the image
image = cv2.imread('/home/josch/catkin_ws/src/robotcraft_maze/world/IMG_20240830_151703_343(1)2.jpg', cv2.IMREAD_COLOR)
if image is None:
    print("Could not open or find the image!")
    exit()

# Convert the image to grayscale
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Define a threshold for what is considered "dark"
dark_threshold = 10  # Adjust this value as needed

# Calculate the number of cells in the grid
grid_width = gray_image.shape[1] // CELL_SIZE
grid_height = gray_image.shape[0] // CELL_SIZE

# Initialize the matrix M with 0 (all cells are initially empty)
M = np.zeros((grid_height, grid_width), dtype=int)

# Iterate over all pixels to find dark ones and mark obstacles in the grid
for y in range(gray_image.shape[0]):
    for x in range(gray_image.shape[1]):
        pixel_value = gray_image[y, x]
        if pixel_value < dark_threshold:
            # Determine the corresponding cell in the grid
            grid_x = x // CELL_SIZE
            grid_y = y // CELL_SIZE

            # Mark the grid cell as occupied by an obstacle
            for dy in range(1):
                for dx in range(1):
                    if 0 <= grid_x + dx < grid_width and 0 <= grid_y + dy < grid_height:
                        M[grid_y + dy, grid_x + dx] = 1

# Print the resulting grid matrix
print("Grid Matrix (0: empty, 1: obstacle):")
print(M)

np.savetxt('/home/josch/catkin_ws/src/robotcraft_maze/world/grid_matrix.txt', M, fmt='%d')

# Optionally, display the grayscale image
cv2.imshow("Gray Image", gray_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
