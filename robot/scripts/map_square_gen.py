#!/usr/bin/env/ python
import sys

import cv2
import numpy
import math
numpy.set_printoptions(threshold=sys.maxsize)

debug = True

if debug:
	print('map square generator in DEBUG mode!')

image_dir = '../map'
image_path = '{}/map_fixed_rotation_final.pgm'.format(image_dir)
square_array_path = 'square_array'

if debug:
	image_path = '{}/simulation_map.pgm'.format(image_dir)
	square_array_path = 'square_array_debug'

# Returns the combined distance from one point to the other points
def combined_distance(points, point):
	dist = 0
	for p in points:
		dist += distance(p, point)

	return dist

def distance(p1, p2):
	return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

map_image = cv2.imread(image_path, -1)
map_image = map_image[::-1, :]

#print(map_image[0:13, 13:26].mean())
#print(map_image.shape)
#print(map_image[0:13, 13:26])

squares = []
square_size = 9
square_mean_treshold = 220.0
if len(sys.argv) > 2:
	square_size = int(sys.argv[1])
	square_mean_treshold = float(sys.argv[2])

for i in range(map_image.shape[0]/square_size):
	for j in range(map_image.shape[1]/square_size):
		square_mean = map_image[i*square_size:(i+1)*square_size, j*square_size:(j+1)*square_size].mean()
		if square_mean > square_mean_treshold:
			square_center = [(i*square_size)+square_size/2, (j*square_size)+square_size/2]
			squares.append([square_center[0], square_center[1], square_mean])
			map_image[square_center[0], square_center[1]] = 0


print(squares)
cv2.imshow('Square centers', map_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

map_image = cv2.imread(image_path, -1)
map_image = map_image[::-1, :]

squares = numpy.array(squares)
numpy.savetxt('{}/{}'.format(image_dir, square_array_path), squares)
print("Saving squares array to: {}".format('{}/{}'.format(image_dir, square_array_path)))

navigation_radius = 15
if len(sys.argv) > 3:
	navigation_radius = int(sys.argv[3])

# Calculate the point closest to the center and add it to navigation points
'''
map_center = [map_image.shape[0]/2, map_image.shape[1]/2]
shortest_index = 0
shortest_dist = distance(map_center, squares[shortest_index])
for i in range(1, len(squares))
	dist = distance(map_center, squares[i])
	if dist < shortest_dist:
		shortest_index = i
		shortest_dist = dist
'''

navigation_points = [squares[len(squares)/2]]

candidates = []
while True:
	for center_point in squares:
		combined_distance = 0
		valid = True
		# Check if the center point is in the radius of already checked points
		for p in navigation_points:
			dist = distance(center_point, p)
			if dist < navigation_radius:
				valid = False
				break
			combined_distance += dist

		if not valid:
			continue

		candidates.append((center_point, combined_distance))

	if len(candidates) == 0:
		break

	candidates.sort(key=lambda x: x[1], reverse=True)
	navigation_points.append(candidates[0][0])
	del candidates[:]

for p in navigation_points:
	print(p)
	map_image[int(p[0]), int(p[1])] = 0

cv2.imshow('Square centers', map_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

