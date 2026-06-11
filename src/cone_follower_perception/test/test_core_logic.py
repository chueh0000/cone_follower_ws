import numpy as np
import pytest

from cone_follower_perception.core_logic import extract_depth_roi, project_pixel_to_3d

def test_extract_depth_roi_valid():
    # Create a dummy 100x100 depth map
    depth_map = np.ones((100, 100)) * 10.0  # Background is 10m away
    
    # Place a cone (closer object) in a bounding box
    bbox = [40, 50, 60, 90] # x_min, y_min, x_max, y_max
    
    # Fill the cone area with 5.0m
    depth_map[50:90, 40:60] = 5.0
    
    # Add some noise (NaNs) in the center pixel to test the single-pixel trap
    depth_map[70, 50] = np.nan
    
    # Calculate depth
    depth = extract_depth_roi(depth_map, bbox, roi_ratio=0.5)
    
    # The expected depth should be 5.0, filtering out the NaN
    assert np.isclose(depth, 5.0), f"Expected 5.0, got {depth}"

def test_extract_depth_roi_all_nan():
    depth_map = np.ones((100, 100)) * np.nan
    bbox = [40, 50, 60, 90]
    
    depth = extract_depth_roi(depth_map, bbox)
    assert np.isnan(depth)

def test_project_pixel_to_3d():
    u, v = 320, 240
    depth = 5.0
    fx, fy = 500.0, 500.0
    cx, cy = 320.0, 240.0
    
    # Center pixel should map to X=0, Y=0
    X, Y, Z = project_pixel_to_3d(u, v, depth, fx, fy, cx, cy)
    assert np.isclose(X, 0.0)
    assert np.isclose(Y, 0.0)
    assert np.isclose(Z, 5.0)
    
    # Pixel off-center
    u2, v2 = 420, 340 # 100 pixels right and down
    X2, Y2, Z2 = project_pixel_to_3d(u2, v2, depth, fx, fy, cx, cy)
    
    # X = (420 - 320) * 5.0 / 500.0 = 100 * 5 / 500 = 1.0
    # Y = (340 - 240) * 5.0 / 500.0 = 100 * 5 / 500 = 1.0
    assert np.isclose(X2, 1.0)
    assert np.isclose(Y2, 1.0)
    assert np.isclose(Z2, 5.0)
