#!/usr/bin/env python3
"""Analyze lane detection on captured images - produces debug visualizations."""
import cv2
import numpy as np
import os

SAVE_DIR = '/home/arihant/ros2_ws/tools/captures'

def analyze_image(path, idx):
    frame = cv2.imread(path)
    h, w, _ = frame.shape
    img_center = w / 2
    
    # --- A. HSV COLOR FILTERING (same as lane_keeper.py) ---
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_white = np.array([0, 0, 160])
    upper_white = np.array([180, 80, 255])
    mask_w = cv2.inRange(hsv, lower_white, upper_white)
    
    lower_yellow = np.array([15, 80, 80])
    upper_yellow = np.array([35, 255, 255])
    mask_y = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    mask = cv2.bitwise_or(mask_w, mask_y)
    
    # --- B. ROI (same as lane_keeper.py) ---
    roi_mask = np.zeros_like(mask)
    roi_pts = np.array([[
        (0, int(h * 0.9)),
        (0, int(h * 0.5)),
        (w, int(h * 0.5)),
        (w, int(h * 0.9))
    ]], np.int32)
    cv2.fillPoly(roi_mask, roi_pts, 255)
    masked = cv2.bitwise_and(mask, roi_mask)
    
    # --- C. CENTER CALCULATION ---
    pixels = np.where(masked > 0)
    
    debug = frame.copy()
    
    # Overlay the mask
    color_mask = cv2.cvtColor(masked, cv2.COLOR_GRAY2BGR)
    color_mask[:,:,0] = 0  # Remove blue channel
    color_mask[:,:,2] = 0  # Remove red channel - make it green
    debug = cv2.addWeighted(debug, 0.7, color_mask, 0.5, 0)
    
    # Draw ROI boundary
    cv2.line(debug, (0, int(h*0.5)), (w, int(h*0.5)), (255, 255, 0), 2)
    cv2.line(debug, (0, int(h*0.9)), (w, int(h*0.9)), (255, 255, 0), 2)
    cv2.putText(debug, "ROI TOP", (10, int(h*0.5)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
    cv2.putText(debug, "ROI BOTTOM", (10, int(h*0.9)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
    
    # Draw center line
    cv2.line(debug, (int(img_center), 0), (int(img_center), h), (0, 0, 255), 2)
    cv2.putText(debug, "IMG CENTER", (int(img_center)+5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
    
    if len(pixels[1]) > 50:
        target_center = np.mean(pixels[1])
        error = img_center - target_center
        
        cv2.circle(debug, (int(target_center), int(h*0.7)), 15, (0, 255, 0), -1)
        cv2.putText(debug, f"TARGET ({int(target_center)})", (int(target_center)+20, int(h*0.7)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        
        cv2.arrowedLine(debug, (int(img_center), int(h*0.7)), (int(target_center), int(h*0.7)), (0,255,255), 3)
        
        status = f"Pixels: {len(pixels[1])} | Error: {error:.1f} | Steer: {error*0.025:.3f}"
    else:
        status = f"LANE LOST! Only {len(pixels[1])} pixels detected"
    
    cv2.putText(debug, status, (10, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
    
    # Also save the raw mask
    out_debug = os.path.join(SAVE_DIR, f'analysis_{idx}.png')
    out_mask = os.path.join(SAVE_DIR, f'mask_{idx}.png')
    cv2.imwrite(out_debug, debug)
    cv2.imwrite(out_mask, masked)
    print(f"[{idx}] {status}")
    print(f"    Saved: {out_debug}")

for i in range(1, 4):
    path = os.path.join(SAVE_DIR, f'camera_{i}.png')
    if os.path.exists(path):
        analyze_image(path, i)
    else:
        print(f"[{i}] File not found: {path}")

print("\nDone! Check the analysis_*.png and mask_*.png files.")
