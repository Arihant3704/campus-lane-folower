#!/usr/bin/env python3
"""Re-analyze with FIXED filters."""
import cv2
import numpy as np
import os

SAVE_DIR = '/home/arihant/ros2_ws/tools/captures'

def analyze_fixed(path, idx):
    frame = cv2.imread(path)
    h, w, _ = frame.shape
    img_center = w / 2
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # FIXED White filter
    mask_w = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 40, 255]))
    # FIXED Yellow filter  
    mask_y = cv2.inRange(hsv, np.array([10, 50, 50]), np.array([40, 255, 255]))
    
    mask = cv2.bitwise_or(mask_w, mask_y)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    # ROI
    roi_mask = np.zeros_like(mask)
    roi_pts = np.array([[(0, int(h*0.9)), (0, int(h*0.5)), (w, int(h*0.5)), (w, int(h*0.9))]], np.int32)
    cv2.fillPoly(roi_mask, roi_pts, 255)
    masked = cv2.bitwise_and(mask, roi_mask)
    
    pixels = np.where(masked > 0)
    debug = frame.copy()
    color_mask = cv2.cvtColor(masked, cv2.COLOR_GRAY2BGR)
    color_mask[:,:,0] = 0; color_mask[:,:,2] = 0
    debug = cv2.addWeighted(debug, 0.7, color_mask, 0.5, 0)
    
    cv2.line(debug, (0, int(h*0.5)), (w, int(h*0.5)), (255,255,0), 2)
    cv2.line(debug, (0, int(h*0.9)), (w, int(h*0.9)), (255,255,0), 2)
    cv2.line(debug, (int(img_center), 0), (int(img_center), h), (0,0,255), 2)
    
    if len(pixels[1]) > 50:
        target = np.mean(pixels[1])
        error = img_center - target
        cv2.circle(debug, (int(target), int(h*0.7)), 15, (0,255,0), -1)
        cv2.arrowedLine(debug, (int(img_center), int(h*0.7)), (int(target), int(h*0.7)), (0,255,255), 3)
        status = f"Pixels: {len(pixels[1])} | Error: {error:.1f} | Steer: {error*0.025:.3f}"
    else:
        status = f"LOST! Only {len(pixels[1])} px"
    
    cv2.putText(debug, status, (10, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
    cv2.putText(debug, "FIXED FILTERS", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
    
    out = os.path.join(SAVE_DIR, f'fixed_{idx}.png')
    cv2.imwrite(out, debug)
    
    out_mask = os.path.join(SAVE_DIR, f'fixed_mask_{idx}.png')
    cv2.imwrite(out_mask, masked)
    print(f"[{idx}] {status}")

for i in range(1, 4):
    p = os.path.join(SAVE_DIR, f'camera_{i}.png')
    if os.path.exists(p): analyze_fixed(p, i)
