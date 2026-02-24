import cv2
import numpy as np

img = cv2.imread('1771728719331_aerial.png')
h, w = img.shape[:2]

# The grey roads are typically in the RGB range of ~ [100,100,100] to [160,160,160].
# But let's look at the BGR values from kmeans: [49, 74, 65] was chosen, which is dark.
# Often roads in Satellite imagery of India are light grey. 
# Let's create an explicit color threshold for light/medium grey
lower_grey = np.array([80, 80, 80])
upper_grey = np.array([170, 170, 170])

# To prevent picking up white roofs, we strictly bind to grey.
# We also want the RGB values to be very close to each other (low saturation).
diff_rg = np.abs(img[:,:,2].astype(int) - img[:,:,1].astype(int))
diff_gb = np.abs(img[:,:,1].astype(int) - img[:,:,0].astype(int))
diff_br = np.abs(img[:,:,0].astype(int) - img[:,:,2].astype(int))

grey_mask = (diff_rg < 20) & (diff_gb < 20) & (diff_br < 20)
val_mask = cv2.inRange(img, lower_grey, upper_grey) > 0
mask = (grey_mask & val_mask).astype(np.uint8) * 255

kernel = np.ones((5,5), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

print("Grey mask size:", np.count_nonzero(mask))

img_new = img.copy()

# Color roads black
img_new[mask > 0] = [30, 30, 30]

# Add dashed white lines along the centers
# Distance transform to find centers
dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)

# A simple heuristic to place dashes: local maxima of distance transform
from scipy.ndimage import maximum_filter
local_max = maximum_filter(dist, size=15) == dist
centers = (local_max & (dist > 3)).astype(np.uint8) * 255

pts = np.column_stack(np.where(centers > 0))
# Let's draw small lines at these centers
for y, x in pts[::3]:  # skip some to make it dashed
    cv2.line(img_new, (x-4, y-4), (x+4, y+4), (255, 255, 255), 2)

cv2.imwrite('1771728719331_aerial_indian_map.png', img_new)
print("Finished Indian map conversion.")

