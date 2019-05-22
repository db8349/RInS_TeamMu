#!/usr/bin/env/ python
import sys

import cv2
import numpy
numpy.set_printoptions(threshold=sys.maxsize)

image_path = '../map/map_fixed_rotation_final.pgm'

map_image = cv2.imread(image_path, -1)

print(map_image[0:13, 0:13].mean())
print(map_image.shape)