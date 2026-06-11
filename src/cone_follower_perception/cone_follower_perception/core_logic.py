import numpy as np

class YOLOInferenceWrapper:
    def __init__(self, model_path=None):
        """
        Wrapper for YOLO model (e.g., ultralytics YOLOv8).
        """
        self.model = None
        if model_path:
            from ultralytics import YOLO
            self.model = YOLO(model_path)

    def detect(self, image):
        """
        Takes an image and returns a list of detections.
        Mock implementation.
        
        Args:
            image (np.ndarray): The RGB image.
            
        Returns:
            list of dicts: [{'bbox': [x_min, y_min, x_max, y_max], 'class_id': int, 'confidence': float}]
        """
        if self.model is None:
            return []
            
        results = self.model(image, verbose=False)
        detections = []
        
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Bounding box in [x_min, y_min, x_max, y_max] format
                b = box.xyxy[0].tolist() 
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                
                detections.append({
                    'bbox': b,
                    'class_id': cls_id,
                    'confidence': conf
                })
                
        return detections

def extract_depth_roi(depth_map, bbox, roi_ratio=0.5):
    """
    Extracts a region of interest from the depth map based on the bounding box.
    Uses the bottom-center region to target where the cone touches the ground.
    
    Args:
        depth_map (np.ndarray): 2D array of depth values.
        bbox (list or tuple): [x_min, y_min, x_max, y_max]
        roi_ratio (float): Ratio of the box width/height to use for the ROI.
    
    Returns:
        float: the robust depth estimate (median of the valid pixels in ROI)
    """
    x_min, y_min, x_max, y_max = map(int, bbox)
    
    # Ensure bounds
    h, w = depth_map.shape
    x_min = max(0, x_min)
    y_min = max(0, y_min)
    x_max = min(w, x_max)
    y_max = min(h, y_max)
    
    if x_min >= x_max or y_min >= y_max:
        return np.nan
        
    width = x_max - x_min
    height = y_max - y_min
    
    # Calculate bottom-center ROI
    roi_w = max(1, int(width * roi_ratio))
    roi_h = max(1, int(height * roi_ratio))
    
    roi_x_min = x_min + (width - roi_w) // 2
    roi_x_max = roi_x_min + roi_w
    
    # Focus on the bottom half/quarter
    roi_y_max = y_max
    roi_y_min = max(y_min, y_max - roi_h)
    
    roi = depth_map[roi_y_min:roi_y_max, roi_x_min:roi_x_max]
    
    # Filter out NaNs and infs
    valid_depths = roi[np.isfinite(roi)]
    
    if len(valid_depths) == 0:
        return np.nan
        
    # Using the 20th percentile (assuming closest points are the cone)
    return np.percentile(valid_depths, 20)

def project_pixel_to_3d(u, v, depth, fx, fy, cx, cy):
    """
    Projects a 2D pixel coordinate (u, v) and its depth to a 3D point (X, Y, Z).
    Assumes standard camera frame (Z forward, X right, Y down).
    
    Args:
        u (float): x-coordinate in image.
        v (float): y-coordinate in image.
        depth (float): Distance Z.
        fx, fy, cx, cy (float): Camera intrinsics.
        
    Returns:
        tuple: (X, Y, Z)
    """
    if np.isnan(depth) or np.isinf(depth):
        return (np.nan, np.nan, np.nan)
        
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    z = depth
    
    return (x, y, z)
