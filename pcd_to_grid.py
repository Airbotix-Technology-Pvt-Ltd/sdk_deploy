import numpy as np
import cv2
import os

def pcd_to_2d_map(pcd_file, output_prefix="map", resolution=0.05, min_z=0.2, max_z=1.0):
    print(f"Loading {pcd_file}...")
    
    # Read points directly from the file (faster than complex libs for Fast-LIO output)
    points = []
    with open(pcd_file, 'r') as f:
        data_start = False
        for line in f:
            if line.startswith('DATA'):
                data_start = True
                continue
            if data_start:
                try:
                    vals = [float(v) for v in line.split()[:3]]
                    if len(vals) == 3:
                        # Filter by Z height (only keep walls/obstacles)
                        if min_z < vals[2] < max_z:
                            points.append(vals[:2])
                except:
                    continue
    
    if not points:
        print("No points found in the specified Z range!")
        return

    points = np.array(points)
    
    # Find map boundaries
    min_x, min_y = np.min(points, axis=0) - 1.0
    max_x, max_y = np.max(points, axis=0) + 1.0
    
    width = int((max_x - min_x) / resolution)
    height = int((max_y - min_y) / resolution)
    
    print(f"Generating map: {width}x{height} pixels at {resolution} resolution")
    
    # Initialize empty occupancy grid (255 = free, 0 = occupied)
    grid = np.full((height, width), 255, dtype=np.uint8)
    
    # Map points to pixels
    px = ((points[:, 0] - min_x) / resolution).astype(int)
    py = ((points[:, 1] - min_y) / resolution).astype(int)
    
    # Mark occupied cells
    for x, y in zip(px, py):
        if 0 <= x < width and 0 <= y < height:
            grid[height - 1 - y, x] = 0  # Invert Y for image coordinates
    
    # Save the PGM image
    pgm_file = f"{output_prefix}.pgm"
    cv2.imwrite(pgm_file, grid)
    print(f"Map image saved to {pgm_file}")
    
    # Save the YAML metadata for Nav2
    yaml_file = f"{output_prefix}.yaml"
    with open(yaml_file, 'w') as f:
        f.write(f"image: {pgm_file}\n")
        f.write(f"resolution: {resolution}\n")
        f.write(f"origin: [{min_x}, {min_y}, 0.0]\n")
        f.write(f"negate: 0\n")
        f.write(f"occupied_thresh: 0.65\n")
        f.write(f"free_thresh: 0.196\n")
    print(f"Map metadata saved to {yaml_file}")

if __name__ == "__main__":
    pcd_path = "/home/lite3/work/Lite3Robot/Lite3_sdk_deploy/simulation_ascii.pcd"
    if os.path.exists(pcd_path):
        pcd_to_2d_map(pcd_path)
    else:
        print(f"Could not find {pcd_path}!")
