
    
    
#d2p.py
import numpy as np
import open3d as o3d
import cv2
import argparse
import os
import torch
import gzip
import matplotlib.pyplot as plt
from cam_cal import CameraCalibrator
import yaml

class PointCloudGenerator:
    
    def __init__(self, save_path=None, camera_matrix=None, dist_coeffs=None):
        if save_path is None:   
            self.save_path = './zdata/results/'
        else:
            self.save_path = save_path
            
        # Ensure save path exists
        os.makedirs(self.save_path, exist_ok=True)
        
        # Camera calibration
        self.calibrator = CameraCalibrator()
        try:
            self.calibrator.load_calibration('./chessboard_calibration.npz')
            self.camera_params = self.calibrator.load_camera_params('./Calibration_result_d455.yaml')
            self.calibration_loaded = True
        except Exception as e:
            print(f"Warning: Could not load calibration: {e}")
            self.calibration_loaded = False
        
        # Default parameters
        self.scale_factor = 2.0
        self.min_depth = 0.1
        self.max_depth = 5.0
        self.z_offset = 2.0
        self.y_offset = 0.7
        
    def calculate_view_angle(self, image_path):
        """Calculate camera view angle using chessboard detection"""
        if not self.calibration_loaded:
            print("Warning: Calibration not loaded, using default angle (3 degrees)")
            return 3.0
            
        try:
            # Detect chessboard and calculate pose
            success, rvec, tvec = self.calibrator.measure_board_pose(image_path)
            if not success:
                print("Warning: Could not detect chessboard, using default angle (3 degrees)")
                return 3.0
            
            # Convert rotation vector to matrix
            rmat, _ = cv2.Rodrigues(rvec)
            
            # Calculate pitch angle (y-axis rotation)
            pitch = np.arctan2(-rmat[2,0], np.sqrt(rmat[2,1]**2 + rmat[2,2]**2))
            pitch_deg = np.degrees(pitch)
            
            print(f"Measured pitch angle: {pitch_deg:.2f} degrees")
            return abs(pitch_deg)
            
        except Exception as e:
            print(f"Error in view angle calculation: {e}")
            print("Using default angle (3 degrees)")
            return 3.0

    def create_point_cloud(self, 
                          depth_map: np.ndarray, 
                          image_path: str,
                          real_distance: float = 1000.0,
                          min_depth: float = 0.01,
                          max_depth: float = 10.0,
                          save_name: str = None) -> o3d.geometry.PointCloud:
        """Create a point cloud from depth map and RGB image"""
        # Debug: Print input depth map statistics
        print("\nInput Depth Map Statistics:")
        print(f"Shape: {depth_map.shape}")
        print(f"Min value: {np.min(depth_map)}")
        print(f"Max value: {np.max(depth_map)}")
        print(f"Mean value: {np.mean(depth_map)}")
        print(f"Non-zero points: {np.count_nonzero(depth_map)}")
        
        # Load and check image
        bgr_image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if bgr_image is None:
            raise FileNotFoundError(f"Could not find image: {image_path}")
        
        # Ensure depth map and image have same dimensions
        if bgr_image.shape[:2] != depth_map.shape:
            print(f"Warning: Image shape {bgr_image.shape[:2]} != depth map shape {depth_map.shape}")
            print("Resizing depth map to match image...")
            depth_map = cv2.resize(depth_map, (bgr_image.shape[1], bgr_image.shape[0]), interpolation=cv2.INTER_LINEAR)
        
        print(f"\nImage shape: {bgr_image.shape}")
        
        # Initialize points collection
        height, width = depth_map.shape
        points = []
        colors = []
        
        # Calculate view angle for perspective correction
        view_angle = self.calculate_view_angle(image_path)
        print(f"Calculated view angle: {view_angle:.2f} degrees")
        
        # Depth calibration from real_distance parameter
        # Adjust scale factors based on this calibration
        scale_factor_z = 100  # Base value, will be adjusted if needed
        
        # Try to compute actual scale factor if a real distance is provided
        if real_distance > 0:
            center_depth = np.median(depth_map[height//3:2*height//3, width//3:2*width//3])
            if center_depth > 0:
                computed_scale = real_distance / center_depth
                if 10 < computed_scale < 1000:  # Sanity check
                    scale_factor_z = computed_scale
                    print(f"Computed scale factor from real distance: {scale_factor_z:.2f}")
        
        scale_factor_x = 555  # Horizontal scaling
        scale_factor_y = 400  # Vertical scaling
        
        # Debug arrays for statistics
        z_values = []
        z_scaled_values = []
        z_adjusted_values = []
        
        # First pass: collect statistics
        for v in range(height):
            for u in range(width):
                if depth_map[v, u] > 0:  # Only process non-zero depths
                    z = depth_map[v, u]
                    z_values.append(z)
                    
                    z_scaled = z / scale_factor_z
                    z_scaled_values.append(z_scaled)
                    
                    z_adjusted = z_scaled + self.z_offset
                    z_adjusted_values.append(z_adjusted)
        
        # Print statistics
        if z_values:
            print("\nDepth Processing Statistics:")
            print(f"Original Z - Min: {min(z_values):.3f}, Max: {max(z_values):.3f}, Mean: {np.mean(z_values):.3f}")
            print(f"Scaled Z - Min: {min(z_scaled_values):.3f}, Max: {max(z_scaled_values):.3f}, Mean: {np.mean(z_scaled_values):.3f}")
            print(f"Adjusted Z - Min: {min(z_adjusted_values):.3f}, Max: {max(z_adjusted_values):.3f}, Mean: {np.mean(z_adjusted_values):.3f}")
        
        # Calculate a reasonable depth range based on statistics
        if z_adjusted_values:
            adaptive_min = max(0.001, np.percentile(z_adjusted_values, 1))
            adaptive_max = min(20.0, np.percentile(z_adjusted_values, 99))
            print(f"Adaptive depth range: {adaptive_min:.3f}m to {adaptive_max:.3f}m")
        else:
            adaptive_min, adaptive_max = min_depth, max_depth
        
        # Second pass: create points with adaptive parameters
        for v in range(height):
            for u in range(width):
                if depth_map[v, u] > 0:
                    z = depth_map[v, u]
                    z_scaled = z / scale_factor_z
                    z_adjusted = z_scaled + self.z_offset
                    
                    # Filter by adaptive depth range
                    if adaptive_min <= z_adjusted <= adaptive_max:
                        # Apply view angle correction if available
                        tilt_angle = np.radians(view_angle)
                        
                        # Calculate 3D coordinates
                        y_adjusted = (v/scale_factor_y - self.y_offset) * np.cos(tilt_angle)
                        x_adjusted = (u - width/2) / scale_factor_x
                        
                        points.append([x_adjusted, y_adjusted, z_adjusted])
                        colors.append(bgr_image[v, u] / 255.0)
        
        # Check if any points were generated
        if len(points) == 0:
            print("\nNo points found! Additional debug info:")
            print(f"Z offset: {self.z_offset}")
            print(f"Scale factors - X: {scale_factor_x}, Y: {scale_factor_y}, Z: {scale_factor_z}")
            raise ValueError(f"No points in specified depth range ({adaptive_min}m ~ {adaptive_max}m)")
        
        points = np.array(points, dtype=np.float32)
        colors = np.array(colors, dtype=np.float32)
        
        print(f"\nFinal Results:")
        print(f"Total points: {len(points)}")
        print(f"Depth range: {np.min(points[:, 2]):.3f}m ~ {np.max(points[:, 2]):.3f}m")
        
        # Create point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # Save the point cloud if requested
        if self.save_path and save_name:
            # Create output directories
            npy_path = f'{self.save_path}/pointclouds/{save_name}/'
            os.makedirs(npy_path, exist_ok=True)
            
            # Create visualization of the point cloud
            self.visualize_point_cloud(pcd, f"{self.save_path}/pointcloud_preview.png")
            
            # Save as compressed numpy file with structured data
            structured_data = np.zeros(len(points), dtype=[
                ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')
            ])
            
            structured_data['x'] = points[:, 0]
            structured_data['y'] = points[:, 1]
            structured_data['z'] = points[:, 2]
            structured_data['red'] = colors[:, 0] * 255
            structured_data['green'] = colors[:, 1] * 255
            structured_data['blue'] = colors[:, 2] * 255
            
            file_name = os.path.join(npy_path, f'{save_name}.npy.gz')
            with gzip.GzipFile(file_name, 'w') as f:
                np.save(f, structured_data)
            print(f"Point cloud saved: {file_name}")
            
            # Also save as PLY for easier viewing
            o3d.io.write_point_cloud(
                os.path.join(npy_path, f'{save_name}.ply'), 
                pcd
            )
        
        return pcd

    def calculate_camera_angle_from_intrinsics(self, image_path=None):
        """
        Estimate camera angle using intrinsic parameters from calibration file
        This doesn't require a chessboard to be present in the current image
        """
        if not hasattr(self, 'camera_params') or not self.calibration_loaded:
            print("Warning: No calibration loaded, using default angle (3 degrees)")
            return 3.0
            
        try:
            # Get image dimensions if available
            if image_path:
                import cv2
                img = cv2.imread(image_path)
                if img is not None:
                    height, width = img.shape[:2]
                else:
                    # Default dimensions if image can't be loaded
                    width, height = 1280, 720
            else:
                # Default dimensions if no image provided
                width, height = 1280, 720
                
            # Extract focal length and principal point from camera matrix
            fx = self.camera_params.get('fx', 1933.51)  # Default to values from your file
            fy = self.camera_params.get('fy', 1950.31)
            cx = self.camera_params.get('cx', 661.44)
            cy = self.camera_params.get('cy', 391.99)
            
            # Calculate vertical field of view (FoV)
            vfov = 2 * np.arctan(height / (2 * fy))
            vfov_degrees = np.degrees(vfov)
            
            # Calculate horizontal field of view
            hfov = 2 * np.arctan(width / (2 * fx))
            hfov_degrees = np.degrees(hfov)
            
            # Calculate principal point offset ratio
            # This can indicate if camera is tilted up or down
            cy_ratio = (cy / height) - 0.5
            
            # Estimate tilt based on principal point offset and typical mounting height
            # This assumes camera is mounted somewhat above eye-level
            # Positive tilt means camera is looking downward
            estimated_tilt = 5.0 + (cy_ratio * 20.0)  # 5° base + adjustment from principal point
            
            print(f"Camera Parameters: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
            print(f"Field of View: vertical={vfov_degrees:.1f}°, horizontal={hfov_degrees:.1f}°")
            print(f"Estimated tilt angle: {estimated_tilt:.2f} degrees")
            
            return abs(estimated_tilt)  # Return absolute value of tilt
            
        except Exception as e:
            print(f"Error estimating camera angle from intrinsics: {e}")
            print("Using default angle (3 degrees)")
            return 3.0

    def load_camera_params(self, yaml_file):
        """Load camera parameters from YAML file or directly from NPY files"""
        try:
            import yaml
            with open(yaml_file, 'r') as f:
                params = yaml.safe_load(f)
                return params
        except Exception as e:
            print(f"Could not load YAML file: {e}")
            try:
                # Try to load directly from NPY files
                import numpy as np
                camera_matrix = np.load('camera_matrix.npy')
                dist_coeffs = np.load('dist_coeffs.npy')
                
                # Convert to dictionary format
                params = {
                    'fx': float(camera_matrix[0, 0]),
                    'fy': float(camera_matrix[1, 1]),
                    'cx': float(camera_matrix[0, 2]),
                    'cy': float(camera_matrix[1, 2]),
                    'distortion_coefficients': dist_coeffs.tolist()
                }
                return params
            except Exception as e2:
                print(f"Failed to load from NPY files: {e2}")
                # Return default parameters based on your calibration data
                return {
                    'fx': 1933.51,
                    'fy': 1950.31,
                    'cx': 661.44,
                    'cy': 391.99,
                    'distortion_coefficients': [[-9.93, 597.35, -0.082, 0.032, -12562.32]]
                }

    def create_wall_point_cloud(self, 
                        depth_map, 
                        image_path, 
                        wall_mask, 
                        save_name=None,
                        normal_vectors=None):  # Add normal_vectors parameter
        """
        Create point cloud specifically for wall analysis with improved angle estimation
        and consistent normal vector handling
        """
        # Debug: Print input statistics
        print("\nWall Point Cloud Input Statistics:")
        print(f"Depth map shape: {depth_map.shape}")
        print(f"Wall mask shape: {wall_mask.shape}")
        print(f"Wall mask non-zero points: {np.count_nonzero(wall_mask)}")
        
        # Check if mask is valid
        if np.count_nonzero(wall_mask) == 0:
            raise ValueError("Wall mask is empty - no wall area detected")
        
        # Load BGR image for point cloud generation
        bgr_image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if bgr_image is None:
            raise FileNotFoundError(f"Could not find image: {image_path}")
        
        # Make sure all arrays have consistent dimensions
        height, width = bgr_image.shape[:2]
        
        # Resize mask and depth map if needed
        if wall_mask.shape != (height, width):
            wall_mask = cv2.resize(wall_mask.astype(np.uint8), (width, height), 
                                interpolation=cv2.INTER_NEAREST).astype(bool)
            
        if depth_map.shape != (height, width):
            depth_map = cv2.resize(depth_map, (width, height), 
                                interpolation=cv2.INTER_LINEAR)
        
        # Save a visualization of what pixels are included in the wall mask
        mask_vis = np.zeros_like(bgr_image)
        for v in range(height):
            for u in range(width):
                if wall_mask[v, u]:
                    mask_vis[v, u] = bgr_image[v, u]
        cv2.imwrite('./zdata/results/included_pixels.png', mask_vis)
        
        # Initialize point collection
        points = []
        colors = []
        
        # Get camera parameters for better perspective correction
        # Use our improved camera angle estimation
        view_angle = self.calculate_camera_angle_from_intrinsics(image_path)
        tilt_angle = np.radians(view_angle)
        
        # Scale factors based on calibration data
        # fx and fy can help determine better scale factors
        if hasattr(self, 'camera_params') and self.camera_params:
            fx = self.camera_params.get('fx', 1933.51)
            fy = self.camera_params.get('fy', 1950.31)
            # Use focal length to inform scale factors
            scale_factor_x = fx / 3.5
            scale_factor_y = fy / 5.0
        else:
            scale_factor_x = 555
            scale_factor_y = 400
        
        scale_factor_z = 100
        
        # Debug arrays
        z_values = []
        z_scaled_values = []
        z_adjusted_values = []
        
        # First pass: collect statistics for wall points only
        for v in range(height):
            for u in range(width):
                if depth_map[v, u] > 0 and wall_mask[v, u]:
                    z = depth_map[v, u]
                    z_values.append(z)
                    
                    z_scaled = z / scale_factor_z
                    z_scaled_values.append(z_scaled)
                    
                    z_corrected = z_scaled * np.cos(tilt_angle)
                    z_adjusted = z_corrected + self.z_offset
                    z_adjusted_values.append(z_adjusted)
        
        # Print statistics
        if z_values:
            print("\nWall Depth Processing Statistics:")
            print(f"Original Z - Min: {min(z_values):.3f}, Max: {max(z_values):.3f}")
            print(f"Scaled Z - Min: {min(z_scaled_values):.3f}, Max: {max(z_scaled_values):.3f}")
            print(f"Adjusted Z - Min: {min(z_adjusted_values):.3f}, Max: {max(z_adjusted_values):.3f}")
            
            # Calculate adaptive depth range for wall points
            wall_z_min = max(0.001, np.percentile(z_adjusted_values, 1))
            wall_z_max = min(20.0, np.percentile(z_adjusted_values, 99))
            print(f"Wall depth range: {wall_z_min:.3f}m to {wall_z_max:.3f}m")
        else:
            print("Warning: No valid wall depth points found!")
            wall_z_min, wall_z_max = 0.001, 20.0
        
        # Second pass: create points with improved camera-based corrections
        for v in range(height):
            for u in range(width):
                if depth_map[v, u] > 0 and wall_mask[v, u]:
                    z = depth_map[v, u]
                    z_scaled = z / scale_factor_z
                    
                    # Apply corrections for wall orientation
                    z_corrected = z_scaled * np.cos(tilt_angle)
                    
                    # Improved coordinate calculations using camera parameters
                    # Center coordinates based on principal point
                    cx = width/2
                    cy = height/2
                    if hasattr(self, 'camera_params') and self.camera_params:
                        cx = self.camera_params.get('cx', width/2)
                        cy = self.camera_params.get('cy', height/2)
                    
                    # Calculate coordinates with better camera model
                    x_adjusted = (u - cx) / scale_factor_x
                    y_adjusted = (v - cy) / scale_factor_y
                    
                    # Apply tilt correction
                    y_adjusted = y_adjusted * np.cos(tilt_angle) - self.y_offset
                    
                    # Final adjusted Z value with tilt correction
                    z_adjusted = z_corrected + self.z_offset
                    
                    # Filter by adaptive wall depth range
                    if wall_z_min <= z_adjusted <= wall_z_max:
                        points.append([x_adjusted, y_adjusted, z_adjusted])
                        colors.append(bgr_image[v, u] / 255.0)
        
        if len(points) == 0:
            raise ValueError("No points found in wall region after depth filtering")

        points = np.array(points, dtype=np.float32)
        colors = np.array(colors, dtype=np.float32)
        
        print(f"\nWall Point Cloud Results:")
        print(f"Total wall points: {len(points)}")
        print(f"Wall depth range: {np.min(points[:, 2]):.3f}m ~ {np.max(points[:, 2]):.3f}m")
        
        # Create point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # Estimate normals for wall analysis
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        
        # Orient normals consistently (toward the camera)
        pcd.orient_normals_towards_camera_location(camera_location=np.array([0, 0, 0]))
        
        # Remove outliers for cleaner wall surface
        clean_pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        
        # Additional step: Try to fit a plane to the wall points
        plane_model, inliers = clean_pcd.segment_plane(
            distance_threshold=0.02,
            ransac_n=3,
            num_iterations=1000
        )
        
        # Extract plane normal and update our understanding of the wall orientation
        a, b, c, d = plane_model
        plane_normal = np.array([a, b, c])
        wall_angle = np.arccos(np.dot(plane_normal, np.array([0, 0, 1])))
        wall_angle_deg = np.degrees(wall_angle)
        print(f"Detected wall orientation: {wall_angle_deg:.2f}° from vertical")
        
        # Extract only the points that belong to the main plane
        plane_pcd = clean_pcd.select_by_index(inliers)
        
        # Create a KD tree for efficient nearest neighbor search
        pcd_tree = o3d.geometry.KDTreeFlann(plane_pcd)
        
        # Add normal vectors to the point cloud if provided
        if normal_vectors is not None and len(normal_vectors) > 0:
            # Print statistics for the provided normal vectors
            all_normals = np.array([v['normal'] for v in normal_vectors])
            print(f"  Normal vector stats: min={np.min(all_normals, axis=0)}, max={np.max(all_normals, axis=0)}")
            print(f"  Average normal: {np.mean(all_normals, axis=0)}")
            
            # Create point cloud normals array
            normals_array = np.asarray(plane_pcd.normals).copy()
            
            # Dictionary to store which points have user-defined normals
            normal_point_indices = {}
            
            # Convert 2D grid points to 3D world coordinates
            for i, vec in enumerate(normal_vectors):
                # Extract center and normal from the grid cell
                center_x, center_y = vec['center']
                nx, ny, nz = vec['normal']
                depth = vec['depth']
                
                # Convert to 3D world coordinates - same transformation as point cloud generation
                cx = width/2
                cy = height/2
                if hasattr(self, 'camera_params') and self.camera_params:
                    cx = self.camera_params.get('cx', width/2)
                    cy = self.camera_params.get('cy', height/2)
                    
                # Convert center point to 3D (match the same transformation as the point cloud)
                x_world = (center_x - cx) / scale_factor_x
                y_world = (center_y - cy) / scale_factor_y * np.cos(tilt_angle) - self.y_offset
                z_world = (depth / scale_factor_z) * np.cos(tilt_angle) + self.z_offset
                
                # Find the nearest point in the point cloud to this world coordinate
                center_3d = np.array([x_world, y_world, z_world])
                
                # Use KD tree to find nearest point
                _, idx, _ = pcd_tree.search_knn_vector_3d(center_3d, 1)
                if idx:
                    # Store the original normal for this point
                    normal_point_indices[idx[0]] = (nx, ny, nz)
        
            # Initialize a new open3d point cloud with only the points where we have grid-based normals
            selected_indices = list(normal_point_indices.keys())
            selected_pcd = plane_pcd.select_by_index(selected_indices)
            
            # Set the normals for these points
            selected_normals = []
            for idx in selected_indices:
                selected_normals.append(normal_point_indices[idx])
            
            selected_pcd.normals = o3d.utility.Vector3dVector(np.array(selected_normals))
            
            # Save the point cloud with original number of grid-based normal vectors
            print(f"Adding {len(normal_vectors)} normal vectors to point cloud")
            
            # Save this selected point cloud as the final result
            plane_pcd = selected_pcd
        
        # Save the wall point cloud
        if save_name:
            npy_path = f'./zdata/ptsave/{save_name}/'
            os.makedirs(npy_path, exist_ok=True)
            
            # Get the final points and colors
            final_points = np.asarray(plane_pcd.points)
            final_colors = np.asarray(plane_pcd.colors)
            final_normals = np.asarray(plane_pcd.normals)
            
            # Create structured array with additional normal vector information
            structured_data = np.zeros(len(final_points), dtype=[
                ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'),
                ('normal_x', '<f4'), ('normal_y', '<f4'), ('normal_z', '<f4')
            ])
            
            structured_data['x'] = final_points[:, 0]
            structured_data['y'] = final_points[:, 1]
            structured_data['z'] = final_points[:, 2]
            structured_data['red'] = final_colors[:, 0] * 255
            structured_data['green'] = final_colors[:, 1] * 255
            structured_data['blue'] = final_colors[:, 2] * 255
            structured_data['normal_x'] = final_normals[:, 0]
            structured_data['normal_y'] = final_normals[:, 1]
            structured_data['normal_z'] = final_normals[:, 2]
            
            # Save as compressed numpy file
            file_name = os.path.join(npy_path, f'{save_name}_wall.npy.gz')
            with gzip.GzipFile(file_name, 'w') as f:
                np.save(f, structured_data)
            print(f"Wall point cloud saved: {file_name}")
            
            # Also save as PLY for easier viewing
            o3d.io.write_point_cloud(
                os.path.join(npy_path, f'{save_name}_wall.ply'), 
                plane_pcd
            )
        
        return plane_pcd
    
    def visualize_point_cloud(self, pcd, output_path):
        """Create a basic visualization of the point cloud"""
        try:
            # Create a visualization
            vis = o3d.visualization.Visualizer()
            vis.create_window(visible=False)
            vis.add_geometry(pcd)
            
            # Set some default view parameters
            view_control = vis.get_view_control()
            view_control.set_zoom(0.8)
            view_control.set_front([0, 0, -1])
            view_control.set_up([0, -1, 0])
            
            # Render and capture image
            vis.poll_events()
            vis.update_renderer()
            vis.capture_screen_image(output_path)
            vis.destroy_window()
            
            print(f"Point cloud visualization saved to {output_path}")
        except Exception as e:
            print(f"Error creating point cloud visualization: {e}")
            
    def calculate_scale_factor(self, depth_map, real_distance):
        """
        Calculate scale factor using known real distance
        
        Args:
            depth_map: Depth map
            real_distance: Actual distance in mm
        """
        # Calculate average depth in center region
        center_region = depth_map[
            depth_map.shape[0]//3:2*depth_map.shape[0]//3,
            depth_map.shape[1]//3:2*depth_map.shape[1]//3
        ]
        depth_value = np.median(center_region)
        
        # Calculate scale factor
        if depth_value > 0:
            scale_factor = real_distance / depth_value
            
            print(f"Depth value at center: {depth_value}")
            print(f"Real distance: {real_distance}mm")
            print(f"Calculated scale factor: {scale_factor}")
            
            return scale_factor
        else:
            print("Warning: Could not calculate scale factor, depth value is zero")
            return 100  # Default value

    def depth_correction(self, depth_map, focal_length):
        """Apply camera parameter corrections to depth map"""
        height, width = depth_map.shape
        cx, cy = width/2, height/2
        
        # Create pixel coordinate grid
        x, y = np.meshgrid(np.arange(width), np.arange(height))
        x = x - cx
        y = y - cy
        
        # Calculate radial distance
        r = np.sqrt(x**2 + y**2)
        
        # Apply optical center angle correction
        angle_factor = focal_length / np.sqrt(focal_length**2 + x**2 + y**2)
        
        return depth_map * angle_factor
    
    