import cv2
import numpy as np

print("Loading image...")
img = cv2.imread('1771728719331_aerial.png')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

print("Creating road mask based on HSV...")
# Adjust thresholds based on a typical grey/concrete road. Adjust if needed.
# H: 0-180 (all hue since grey has low saturation)
# S: 0-40 (low saturation for greys/whites)
# V: 80-220 (not too dark, not too bright to avoid pure white roofs or black shadows)
lower = np.array([0, 0, 80])
upper = np.array([180, 40, 220])
mask = cv2.inRange(hsv, lower, upper)

print("Morphological cleaning...")
# Clean up the mask
kernel = np.ones((7,7), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
# Close small holes
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

print("Mask size:", np.count_nonzero(mask))

# If mask is extremely small, we might have wrong HSV
if np.count_nonzero(mask) < 10000:
    print("Warning: mask is very small!")

# 1. Paint road black
img_new = img.copy()
img_new[mask > 0] = [30, 30, 30] # BGR dark grey/black

# 2. Extract skeleton (centerline)
print("Extracting centerline...")
# We use morphological thinning/skeletonization
# Since scikit-image may not be available immediately, we'll try OpenCV distance transform approach
dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
# Find the ridge line (approximate skeleton)
thresh = dist.max() * 0.4 # Adjust ratio
ridge_mask = (dist > thresh).astype(np.uint8) * 255

# Or simpler: contour extraction and draw dashed line along approximated midline
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print("Found", len(contours), "road segments")

# Draw dashed lines
line_length = 20
gap_length = 20
thickness = 3

for cnt in contours:
    # Filter small specs
    if cv2.contourArea(cnt) < 500:
        continue
    
    # Fit a spline or just draw along the skeleton
    # For simplicity, we can skeletonize properly if skimage is ready
    try:
        from skimage.morphology import skeletonize
        skel = skeletonize(mask > 0)
        skel_pts = np.column_stack(np.where(skel))
        # skel_pts is (y,x)
        # It's hard to order them perfectly without graph traversal, 
        # so drawing dashed lines over unordered points won't look perfectly dashed, it'll look like dots.
        # Let's try drawing every 5th point to make a dotted line
        filtered_pts = skel_pts[::15] 
        for y, x in filtered_pts:
            cv2.circle(img_new, (x, y), thickness, (255, 255, 255), -1)
    except Exception as e:
        print("Fallback dot method due to:", e)
        # Fallback if skimage fails
        dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
        # Local maxima
        kernel = np.ones((5,5), np.uint8)
        local_max = cv2.dilate(dist, kernel) == dist
        centers = np.logical_and(local_max, dist > 10)
        pts = np.column_stack(np.where(centers))
        for y, x in pts:
            cv2.circle(img_new, (x, y), thickness, (255, 255, 255), -1)

print("Saving modified map...")
cv2.imwrite('1771728719331_aerial_indian_road.png', img_new)
print("Done. Check 1771728719331_aerial_indian_road.png")

