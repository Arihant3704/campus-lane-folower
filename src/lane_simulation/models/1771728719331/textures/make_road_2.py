import cv2
import numpy as np
import json
from shapely.geometry import shape

img = cv2.imread('1771728719331_aerial.png')
h, w = img.shape[:2]
mask = np.zeros((h, w), dtype=np.uint8)

print("Reading geojson...")
with open('../buildings.geojson', 'r') as f:
    geojson = json.load(f)

# Need bounding box from .sdf or geojson to map coordinates to pixels
# Usually Gazebo model scale is 1 pixel = 1 meter or similar, but the bounds from geojson:
# Lon: 75.116119 - 75.128494
# Lat: 15.363487 - 15.374554
# We don't have exact coordinate mapping in aerial texture without trial.
# Let's try an unsupervised color segmentation instead.

# Let's cluster the image to find the largest 'grey' class
Z = img.reshape((-1,3))
Z = np.float32(Z)

# Define criteria and apply kmeans()
print("Clustering pixels... (this takes a moment)")
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
K = 8
ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)

center = np.uint8(center)
# The road is likely one of the grey clusters
# Calculate grey-ness (std dev of BGR channels)
grey_ness = np.std(center, axis=1)
# And we want average brightness between 50 and 200
avg_val = np.mean(center, axis=1)

# Sort by greyish (lowest std dev) and valid brightness
best_cluster = -1
min_std = 999
for i in range(K):
    if 50 < avg_val[i] < 200 and grey_ness[i] < min_std:
        best_cluster = i
        min_std = grey_ness[i]

print(f"Selected cluster {best_cluster} with BGR {center[best_cluster]} as roads")

mask = (label == best_cluster).reshape(h, w).astype(np.uint8) * 255

# Try coloring it and check
img_new = img.copy()
img_new[mask > 0] = [30, 30, 30]

cv2.imwrite('cluster_road.png', img_new)
print("Done!")

