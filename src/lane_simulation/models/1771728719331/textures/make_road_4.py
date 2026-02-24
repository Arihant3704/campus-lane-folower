import cv2
import numpy as np

img = cv2.imread('1771728719331_aerial.png')
# Many satellite images have yellowish/brownish dirt roads too.
# The user asked: "where ever there is road make a black road with one white line"
# We know the roads from visually looking at satellite images usually form long connected components.
# Let's try to isolate the connected paths using edge detection, and then fill them.
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

blur = cv2.GaussianBlur(gray, (5, 5), 0)
# Canny to find edges of roads (and buildings)
edges = cv2.Canny(blur, 50, 150)

# The most accurate way is creating a new mask manually or using the K-means cluster that actually represented the roads.
# Cluster 0 was [49,74,65] which is very dark green/brown. Let's look at cluster 1,2..
Z = img.reshape((-1,3)).astype(np.float32)
crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
_, label, center = cv2.kmeans(Z, 6, None, crit, 10, cv2.KMEANS_RANDOM_CENTERS)

# Instead of color, I will make the whole map "Indian Road" format anywhere the texture is bright and has low saturation
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# Brightness > 100, Saturation < 50
road_mask = (hsv[:,:,1] < 60) & (hsv[:,:,2] > 70) & (hsv[:,:,2] < 200)

mask = road_mask.astype(np.uint8) * 255
kernel = np.ones((7,7), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)

# Filter by area to keep only large components (roads)
cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
clean_mask = np.zeros_like(mask)
for c in cnts:
    if cv2.contourArea(c) > 2000:
        cv2.drawContours(clean_mask, [c], -1, 255, -1)

img[clean_mask > 0] = [30, 30, 30]

dist = cv2.distanceTransform(clean_mask, cv2.DIST_L2, 5)
thresh = np.max(dist) * 0.3
skel = (dist > thresh).astype(np.uint8) * 255

pts = np.column_stack(np.where(skel > 0))
for i in range(0, len(pts), 20):
    y, x = pts[i]
    cv2.line(img, (x-5, y-5), (x+5, y+5), (255, 255, 255), 2)

cv2.imwrite('1771728719331_aerial.png', img)
print("Updated original aerial.png")

