


# sd1.py 
import torch
import cv2
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')  
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d
from scipy.ndimage import binary_erosion, binary_dilation
import os
import gzip
import yaml
import sys
import argparse

from ament_index_python.packages import get_package_share_directory

# Segment Anything v2 (SAM 2) related
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator

# Depth Anything v2 related
from depth_utils import load_depth_model, infer_depth
from d2p import PointCloudGenerator

def main():
    try:
        # ROS2 패키지의 'share' 디렉토리 절대 경로를 찾습니다.
        package_share_dir = get_package_share_directory('realsense_robot_control')
        # share 디렉토리 안의 'pth' 폴더 경로를 설정합니다.
        pth_dir = os.path.join(package_share_dir, 'pth')
        print(f"Successfully found model directory via ament: {pth_dir}")
    except Exception as e:
        # ROS2 환경이 아닐 경우를 대비한 대체 경로 (필수는 아님)
        print(f"Could not find package via ament, falling back to relative path. Error: {e}")
        pth_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pth')
    parser = argparse.ArgumentParser(description='Process an image to detect equipment parts and generate 3D point clouds with normal vectors.')
    parser.add_argument('image_path', type=str, help='Path to the input image file')
    parser.add_argument('--output_dir', type=str, default='./zdata', help='Directory to save output files')
    parser.add_argument('--device', type=str, default='cuda' if torch.cuda.is_available() else 'cpu', 
                        help='Device to use for computation (cuda/cpu)')
    parser.add_argument('--grid_size', type=float, default=10.0, 
                        help='Size of grid cells in cm for normal vector extraction')
    
    args = parser.parse_args()
    
    # Validate input image path
    if not os.path.exists(args.image_path):
        print(f"Error: Input image not found at path: {args.image_path}")
        sys.exit(1)
    
    # Extract file name without extension to use as save_name
    input_name = os.path.splitext(os.path.basename(args.image_path))[0]
    save_name = input_name
    
    # Create output directories
    os.makedirs(args.output_dir, exist_ok=True)
    os.makedirs(f'{args.output_dir}/ptsave', exist_ok=True)
    os.makedirs(f'{args.output_dir}/normal_vectors', exist_ok=True)
    if save_name:
        os.makedirs(f'{args.output_dir}/ptsave/{save_name}', exist_ok=True)
        os.makedirs(f'{args.output_dir}/normal_vectors/{save_name}', exist_ok=True)

   
    # 1. Load models
  
    device = torch.device(args.device)
    print(f"Using device: {device}")

    # (1) SAM 2 model loading
    print("Loading SAM 2 model")
    sam_checkpoint_path = os.path.join(pth_dir, "sam2.1_hiera_large.pt")

    # ament_index를 사용해 'SAM-2' 패키지의 공유 디렉토리에서 .yaml 설정 파일 경로를 찾습니다.
    try:
        sam2_share_dir = get_package_share_directory('SAM-2')
        model_cfg = os.path.join(pth_dir, 'sam2.1_hiera_l.yaml')
        print(f"Found SAM-2 config file at: {model_cfg}")
        if not os.path.exists(model_cfg):
            print(f"Error: SAM-2 config file not found at expected path!")
            sys.exit(1)
    except Exception as e:
        print(f"Could not find 'SAM-2' package share directory: {e}")
        print("Please ensure the 'SAM-2' package is sourced correctly using 'source install/setup.bash'")
        sys.exit(1)
        
    # sam2 API를 사용해 모델을 빌드합니다.
    sam_model = build_sam2(model_cfg, sam_checkpoint_path)
    sam_model.to(device)

    # sam2 버전에 맞는 Predictor를 사용합니다.
    predictor = SAM2ImagePredictor(sam_model)

    # (2) Depth Anything v2 model loading
    print("Loading Depth Anything v2 model")
    depth_checkpoint = os.path.join(pth_dir, "depth_anything_v2_vitl.pth")
    depth_model = load_depth_model(depth_checkpoint, device=device)

    
    # 2. Load image
 
    print(f"Loading image: {args.image_path}")
    original_image = cv2.imread(args.image_path)
    if original_image is None:
        print(f"Error: Could not load image at {args.image_path}")
        sys.exit(1)
        
    # BGR to RGB conversion
    if original_image.shape[2] == 3:
        image_rgb = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
    else:
        image_rgb = original_image

    height, width = original_image.shape[:2]
    print(f"Image dimensions: {width}x{height}")

    # Helper function to ensure integer coordinates for OpenCV
    def ensure_int_bbox(bbox):
        """Convert bbox coordinates to integers for OpenCV compatibility"""
        return [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])]

    
    # 3. STAGE 1: Detect largest mask in full image as ROI
  
    print("\n=== STAGE 1: Detecting ROI using SAM ===")
    print("ROI Criteria:")
    print("  - Must not touch image edges (5% margin)")
    print("  - Must be near image center (within 30% of image size)")
    print("  - Must be significant in size (>5% of image area)")
    print("Setting up SAM for ROI detection")
    
    # Resize image if needed to reduce memory consumption
    max_dim = 1024  # Maximum dimension
    h, w = image_rgb.shape[:2]
    scale = 1.0
    if max(h, w) > max_dim:
        scale = max_dim / max(h, w)
        new_h, new_w = int(h * scale), int(w * scale)
        image_for_sam = cv2.resize(image_rgb, (new_w, new_h))
        print(f"Resized image from {h}x{w} to {new_h}x{new_w} for SAM processing")
    else:
        image_for_sam = image_rgb

    # Free up CUDA memory
    torch.cuda.empty_cache()

    predictor.set_image(image_for_sam)

    # Configuration for ROI detection (using same settings as original)
    mask_generator = SAM2AutomaticMaskGenerator(
        sam_model,
        points_per_side=32,            # High point density to catch all parts
        pred_iou_thresh=0.88,          # Higher threshold for better quality masks
        stability_score_thresh=0.92,   # Higher stability for distinct parts
        crop_n_layers=1,               # Single layer
        crop_n_points_downscale_factor=2,  # Less downscaling to capture details
        min_mask_region_area=2000,     # Lower threshold to capture smaller parts
        # output_mode="binary_mask",     # Binary masks to save memory
        box_nms_thresh=0.7,            # Higher NMS to avoid merging distinct parts
        point_grids=None               # Use uniform grid sampling
    )

    print("Generating SAM masks for ROI detection")
    try:
        # Use image_for_sam (possibly resized) instead of original
        roi_masks = mask_generator.generate(image_for_sam)
        print(f"Generated {len(roi_masks)} masks for ROI detection")
        
        # If we resized the image, adjust the masks to match original image size
        if scale < 1.0:
            print("Rescaling masks to match original image size")
            for mask_info in roi_masks:
                # Rescale segmentation mask
                mask = mask_info['segmentation']
                rescaled_mask = cv2.resize(
                    mask.astype(np.uint8), 
                    (width, height), 
                    interpolation=cv2.INTER_NEAREST
                ).astype(bool)
                mask_info['segmentation'] = rescaled_mask
                
                # Rescale bounding box and ensure integer coordinates
                bbox = mask_info['bbox']  # [x, y, w, h]
                mask_info['bbox'] = ensure_int_bbox([
                    bbox[0] / scale,
                    bbox[1] / scale,
                    bbox[2] / scale,
                    bbox[3] / scale
                ])
                
                # Recalculate area
                mask_info['area'] = float(np.sum(rescaled_mask))
    except RuntimeError as e:
        if "CUDA out of memory" in str(e):
            print("CUDA out of memory error. Trying with even smaller image and simpler settings...")
            # Try with even smaller image and simpler settings
            torch.cuda.empty_cache()
            
            # More aggressive resize
            max_dim = 768
            h, w = image_rgb.shape[:2]
            scale = max_dim / max(h, w)
            new_h, new_w = int(h * scale), int(w * scale)
            image_for_sam = cv2.resize(image_rgb, (new_w, new_h))
            print(f"Resized image to {new_h}x{new_w} for SAM processing")
            
            # Memory-efficient settings
            predictor.set_image(image_for_sam)
            mask_generator = SAM2AutomaticMaskGenerator(
                sam_model,
                points_per_side=16,
                pred_iou_thresh=0.88,
                stability_score_thresh=0.92,
                crop_n_layers=0,  # No additional crops
                crop_n_points_downscale_factor=4,
                min_mask_region_area=2000,
                # output_mode="binary_mask"
            )
            
            roi_masks = mask_generator.generate(image_for_sam)
            print(f"Generated {len(roi_masks)} masks with reduced settings")
            
            # Rescale masks to original image size
            for mask_info in roi_masks:
                # Rescale segmentation mask
                mask = mask_info['segmentation']
                rescaled_mask = cv2.resize(
                    mask.astype(np.uint8), 
                    (width, height), 
                    interpolation=cv2.INTER_NEAREST
                ).astype(bool)
                mask_info['segmentation'] = rescaled_mask
                
                # Rescale bounding box and ensure integer coordinates
                bbox = mask_info['bbox']  # [x, y, w, h]
                mask_info['bbox'] = ensure_int_bbox([
                    bbox[0] / scale,
                    bbox[1] / scale,
                    bbox[2] / scale,
                    bbox[3] / scale
                ])
                
                # Recalculate area
                mask_info['area'] = float(np.sum(rescaled_mask))
        else:
            # Re-raise if it's not a memory error
            raise

    # Define function to check if mask is suitable for ROI
    def is_suitable_roi(mask_info, image_center, width, height):
        """
        Check if a mask is suitable to be the ROI:
        - Should not touch image edges
        - Should be near image center
        """
        mask = mask_info['segmentation']
        bbox = mask_info['bbox']
        x, y, w, h = bbox
        
        # Calculate mask centroid
        y_indices, x_indices = np.where(mask)
        if len(y_indices) == 0 or len(x_indices) == 0:
            return False
            
        mask_center_x = np.mean(x_indices)
        mask_center_y = np.mean(y_indices)
        
        # Check 1: Mask should not touch image edges
        edge_margin = min(width, height) * 0.05  # 5% margin from edges
        not_touching_edges = (x > edge_margin and 
                             y > edge_margin and 
                             x + w < width - edge_margin and 
                             y + h < height - edge_margin)
        
        # Check 2: Mask should be near image center
        distance_to_center = np.sqrt((image_center[0] - mask_center_x)**2 + 
                                   (image_center[1] - mask_center_y)**2)
        max_center_distance = min(width, height) * 0.3  # Within 30% of image size from center
        is_central = distance_to_center < max_center_distance
        
        # Check 3: Mask should be significant in size
        min_area = (width * height) * 0.05  # At least 5% of image area
        is_significant = mask_info['area'] > min_area
        
        return not_touching_edges and is_central and is_significant
    
    # Calculate image center
    image_center = (width // 2, height // 2)
    
    # Filter masks to find suitable ROI candidates
    suitable_roi_masks = []
    for mask_info in roi_masks:
        if is_suitable_roi(mask_info, image_center, width, height):
            suitable_roi_masks.append(mask_info)
    
    print(f"Found {len(suitable_roi_masks)} suitable ROI candidates from {len(roi_masks)} total masks")
    
    # Find the largest suitable mask as ROI
    if not suitable_roi_masks:
        print("Warning: No suitable ROI masks found (not touching edges and near center).")
        print("Falling back to largest mask that doesn't touch edges...")
        
        # Try relaxed criteria - just avoid edge-touching masks
        for mask_info in roi_masks:
            bbox = mask_info['bbox']
            x, y, w, h = bbox
            edge_margin = min(width, height) * 0.02  # Smaller margin
            if (x > edge_margin and y > edge_margin and 
                x + w < width - edge_margin and y + h < height - edge_margin):
                suitable_roi_masks.append(mask_info)
        
        if not suitable_roi_masks:
            print("Error: No masks found that don't touch edges. Using full image as ROI.")
            roi_mask = np.ones((height, width), dtype=bool)
            roi_bbox = [0, 0, width, height]
            largest_mask_info = {'area': float(width * height)}  # Create dummy mask info
        else:
            # Sort by area and get the largest
            suitable_roi_masks.sort(key=lambda x: x['area'], reverse=True)
            largest_mask_info = suitable_roi_masks[0]
            roi_mask = largest_mask_info['segmentation']
            roi_bbox = largest_mask_info['bbox']
            print(f"Selected largest non-edge mask as ROI with area: {largest_mask_info['area']:.0f}")
    else:
        # Sort suitable masks by area and get the largest
        suitable_roi_masks.sort(key=lambda x: x['area'], reverse=True)
        largest_mask_info = suitable_roi_masks[0]
        roi_mask = largest_mask_info['segmentation']
        roi_bbox = largest_mask_info['bbox']
        
        # Calculate and print ROI properties
        y_indices, x_indices = np.where(roi_mask)
        mask_center_x = np.mean(x_indices)
        mask_center_y = np.mean(y_indices)
        distance_to_center = np.sqrt((image_center[0] - mask_center_x)**2 + 
                                   (image_center[1] - mask_center_y)**2)
        
        print(f"Selected ROI with area: {largest_mask_info['area']:.0f}")
        print(f"ROI center: ({mask_center_x:.0f}, {mask_center_y:.0f})")
        print(f"Distance to image center: {distance_to_center:.0f} pixels")
        print(f"ROI bounding box: {roi_bbox}")

    # Visualize ROI selection process
    roi_vis = original_image.copy()
    roi_debug = original_image.copy()
    
    # Draw all masks that were considered
    for i, mask_info in enumerate(roi_masks[:10]):  # Show up to 10 masks for clarity
        mask = mask_info['segmentation']
        bbox = ensure_int_bbox(mask_info['bbox'])  # Ensure integer coordinates
        x, y, w, h = bbox
        
        # Check if this mask meets ROI criteria
        y_indices, x_indices = np.where(mask)
        if len(y_indices) > 0 and len(x_indices) > 0:
            mask_center_x = np.mean(x_indices)
            mask_center_y = np.mean(y_indices)
            distance_to_center = np.sqrt((image_center[0] - mask_center_x)**2 + 
                                       (image_center[1] - mask_center_y)**2)
            
            # Color based on criteria
            edge_margin = min(width, height) * 0.05
            not_touching_edges = (x > edge_margin and y > edge_margin and 
                                 x + w < width - edge_margin and y + h < height - edge_margin)
            is_central = distance_to_center < min(width, height) * 0.3
            
            if not_touching_edges and is_central:
                color = (0, 255, 0)  # Green for suitable candidates
            elif not_touching_edges:
                color = (255, 255, 0)  # Yellow for non-edge but not central
            else:
                color = (255, 0, 0)  # Red for edge-touching
            
            cv2.rectangle(roi_debug, (x, y), (x + w, y + h), color, 2)
            cv2.putText(roi_debug, f"#{i+1}", (x, y - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    
    # # Draw edge margin boundaries
    # edge_margin = min(width, height) * 0.05
    # cv2.rectangle(roi_debug, 
    #               (int(edge_margin), int(edge_margin)), 
    #               (int(width - edge_margin), int(height - edge_margin)), 
    #               (255, 0, 255), 2)  # Magenta for edge margin
    # cv2.putText(roi_debug, "Edge Margin (5%)", 
    #             (int(edge_margin + 5), int(edge_margin - 5)), 
    #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    
    # # Draw center distance circle
    # max_center_distance = min(width, height) * 0.3
    # cv2.circle(roi_debug, image_center, int(max_center_distance), (0, 255, 255), 2)
    # cv2.putText(roi_debug, "Center Region (30%)", 
    #             (image_center[0] - 80, image_center[1] + int(max_center_distance) + 20), 
    #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    
    # # Draw image center
    # cv2.circle(roi_debug, image_center, 10, (0, 0, 255), -1)
    # cv2.putText(roi_debug, "Center", (image_center[0] + 15, image_center[1]), 
    #            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    # # Add legend
    # legend_y = 30
    # cv2.rectangle(roi_debug, (10, legend_y), (30, legend_y + 15), (0, 255, 0), -1)
    # cv2.putText(roi_debug, "Suitable ROI (center + no edges)", (35, legend_y + 12), 
    #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # cv2.rectangle(roi_debug, (10, legend_y + 25), (30, legend_y + 40), (255, 255, 0), -1)
    # cv2.putText(roi_debug, "No edges (but not central)", (35, legend_y + 37), 
    #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    
    # cv2.rectangle(roi_debug, (10, legend_y + 50), (30, legend_y + 65), (255, 0, 0), -1)
    # cv2.putText(roi_debug, "Touches edges", (35, legend_y + 62), 
    #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    
    # # Save debug visualization
    # cv2.imwrite(f'{args.output_dir}/roi_candidates_debug.jpg', roi_debug)
    
    # Highlight selected ROI if found
    if 'largest_mask_info' in locals():
        roi_overlay = np.zeros_like(original_image)
        roi_overlay[roi_mask] = [0, 255, 0]  # Green color for ROI
        roi_vis = cv2.addWeighted(roi_vis, 0.7, roi_overlay, 0.3, 0)
        x, y, w, h = ensure_int_bbox(roi_bbox)  # Ensure integer coordinates
        cv2.rectangle(roi_vis, (x, y), (x + w, y + h), (0, 255, 0), 3)
        
        # Add ROI info
        cv2.putText(roi_vis, f"Selected ROI", (x, y - 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.putText(roi_vis, f"Area: {largest_mask_info['area']:.0f}", 
                    (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw center line to ROI
        if 'mask_center_x' in locals() and 'mask_center_y' in locals():
            cv2.line(roi_vis, image_center, (int(mask_center_x), int(mask_center_y)), 
                    (0, 255, 255), 2)
            cv2.putText(roi_vis, f"Dist: {distance_to_center:.0f}px", 
                        ((image_center[0] + int(mask_center_x))//2, 
                         (image_center[1] + int(mask_center_y))//2), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    else:
        # No suitable ROI found - show full image as ROI
        cv2.putText(roi_vis, "No suitable ROI found - using full image", 
                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    
    cv2.imwrite(f'{args.output_dir}/roi_detection.jpg', roi_vis)

    
    # 4. Extract ROI from image
  
    print("\nExtracting ROI from image")
    x, y, w, h = ensure_int_bbox(roi_bbox)  # Ensure integer coordinates
    
    # Crop the image to ROI with some padding
    padding = 10
    x_start = max(0, x - padding)
    y_start = max(0, y - padding)
    x_end = min(width, x + w + padding)
    y_end = min(height, y + h + padding)
    
    # Crop the ROI
    roi_image = image_rgb[y_start:y_end, x_start:x_end]
    roi_mask_cropped = roi_mask[y_start:y_end, x_start:x_end]
    
    # Apply mask to get only the ROI content
    roi_image_masked = roi_image.copy()
    
    # Instead of setting to black (0), we use a neutral gray color (128) for the background
    # This prevents SAM from detecting the uniform background as a separate object/part
    # SAM tends to segment large uniform dark regions as objects, which we want to avoid
    background_color = 128  # Mid-gray
    roi_image_masked[~roi_mask_cropped] = background_color
    
    print(f"Set background outside ROI to gray (value={background_color}) to prevent false detections")
    
    # Also create a version with actual black for visualization
    roi_image_black_bg = roi_image.copy()
    roi_image_black_bg[~roi_mask_cropped] = 0
    
    print(f"ROI dimensions: {roi_image_masked.shape[1]}x{roi_image_masked.shape[0]}")
    
    # Save ROI for debugging
    cv2.imwrite(f'{args.output_dir}/roi_extracted.jpg', cv2.cvtColor(roi_image_black_bg, cv2.COLOR_RGB2BGR))

       
    # 4.5. Compute depth map for ROI and create depth-based mask
    
    print("\nComputing depth map for ROI and creating depth-based mask")
    
    # Process depth for the ROI region
    roi_image_for_depth = roi_image.copy()  # Use the extracted ROI image
    
    # Compute depth for ROI
    torch.cuda.empty_cache()
    roi_depth_map = infer_depth(depth_model, roi_image_for_depth)
    
    print(f"ROI depth map shape: {roi_depth_map.shape}")
    print(f"ROI depth range: {np.min(roi_depth_map):.4f} to {np.max(roi_depth_map):.4f}")
    
    # Create depth-based mask for areas with depth > 150
    depth_threshold = 150.0
    depth_based_mask = roi_depth_map > depth_threshold
    
    # Also intersect with the original ROI mask to ensure we stay within bounds
    depth_based_mask = np.logical_and(depth_based_mask, roi_mask_cropped)
    
    print(f"Depth threshold: {depth_threshold}")
    print(f"Pixels with depth > {depth_threshold}: {np.sum(depth_based_mask)}")
    print(f"Total ROI pixels: {np.sum(roi_mask_cropped)}")
    print(f"Percentage of ROI with depth > {depth_threshold}: {100 * np.sum(depth_based_mask) / np.sum(roi_mask_cropped):.1f}%")
    
    # Save debug visualizations
    plt.figure(figsize=(15, 5))
    
    plt.subplot(131)
    plt.imshow(roi_image)
    plt.title('ROI Image')
    plt.axis('off')
    
    plt.subplot(132)
    plt.imshow(roi_depth_map, cmap='viridis')
    plt.colorbar(label='Depth Value')
    plt.title('ROI Depth Map')
    plt.axis('off')
    
    plt.subplot(133)
    plt.imshow(depth_based_mask, cmap='gray')
    plt.title(f'Depth-Based Mask (depth > {depth_threshold})')
    plt.axis('off')
    
    plt.tight_layout()
    plt.savefig(f'{args.output_dir}/roi_depth_analysis.png', dpi=300)
    plt.close()
    
    # Create a depth-filtered image for SAM
    # Set areas with depth <= threshold to neutral gray to prevent SAM from detecting them
    roi_image_depth_filtered = roi_image.copy()
    roi_image_depth_filtered[~depth_based_mask] = 128  # Neutral gray for excluded areas
    
    # Save for debugging
    cv2.imwrite(f'{args.output_dir}/roi_depth_filtered.jpg', 
                cv2.cvtColor(roi_image_depth_filtered, cv2.COLOR_RGB2BGR))
    
    # Check if we have enough valid depth area
    if np.sum(depth_based_mask) < 1000:  # Minimum threshold
        print(f"Warning: Very few pixels ({np.sum(depth_based_mask)}) meet depth threshold.")
        print("Consider lowering the depth threshold or check if depth values are correctly scaled.")
        # Optionally lower the threshold
        depth_threshold = np.percentile(roi_depth_map[roi_mask_cropped], 75)  # Use 75th percentile
        depth_based_mask = roi_depth_map > depth_threshold
        depth_based_mask = np.logical_and(depth_based_mask, roi_mask_cropped)
        print(f"Using adaptive threshold: {depth_threshold:.1f}")
        print(f"Pixels with adaptive threshold: {np.sum(depth_based_mask)}")

   
    # 5. STAGE 2: Detect parts within depth-filtered ROI using SAM
 
    print("\n=== STAGE 2: Detecting parts within depth-filtered ROI using SAM ===")
    print(f"Using depth threshold: {depth_threshold:.1f}")
    print(f"Processing {np.sum(depth_based_mask)} pixels that meet depth criteria")
    
    # Clear CUDA memory before second stage
    torch.cuda.empty_cache()
    
    # Resize ROI if needed for SAM processing
    roi_h, roi_w = roi_image_depth_filtered.shape[:2]
    roi_scale = 1.0
    max_dim = 1024
    
    if max(roi_h, roi_w) > max_dim:
        roi_scale = max_dim / max(roi_h, roi_w)
        new_roi_h, new_roi_w = int(roi_h * roi_scale), int(roi_w * roi_scale)
        roi_for_sam = cv2.resize(roi_image_depth_filtered, (new_roi_w, new_roi_h))
        depth_mask_resized = cv2.resize(depth_based_mask.astype(np.uint8), 
                                       (new_roi_w, new_roi_h), 
                                       interpolation=cv2.INTER_NEAREST).astype(bool)
        roi_depth_resized = cv2.resize(roi_depth_map, (new_roi_w, new_roi_h), 
                                      interpolation=cv2.INTER_LINEAR)
        print(f"Resized ROI from {roi_h}x{roi_w} to {new_roi_h}x{new_roi_w} for SAM processing")
    else:
        roi_for_sam = roi_image_depth_filtered
        depth_mask_resized = depth_based_mask
        roi_depth_resized = roi_depth_map

    # Set the depth-filtered ROI image for SAM
    predictor.set_image(roi_for_sam)

    # Use your updated SAM configuration for part detection within depth-filtered ROI
    mask_generator = SAM2AutomaticMaskGenerator(
        sam_model,
        points_per_side=32,
        pred_iou_thresh=0.70,           # Your updated value
        stability_score_thresh=0.65,    # Your updated value
        crop_n_layers=1,
        crop_n_points_downscale_factor=2,
        min_mask_region_area=500,       # Reduced for smaller parts within ROI
        # output_mode="binary_mask",
        box_nms_thresh=0.85,           # Your updated value
        point_grids=None
    )

    print("Generating SAM masks within depth-filtered ROI")
    try:
        masks = mask_generator.generate(roi_for_sam)
        print(f"Generated {len(masks)} masks within depth-filtered ROI")
        
        # Filter masks to only include those that overlap significantly with depth-based areas
        print("\nFiltering masks to ensure they overlap with depth-filtered areas...")
        valid_masks = []
        for i, mask_info in enumerate(masks):
            mask_seg = mask_info['segmentation']
            
            # Check how much of the mask overlaps with depth-filtered areas
            overlap_with_depth = np.logical_and(mask_seg, depth_mask_resized)
            overlap_ratio = np.sum(overlap_with_depth) / np.sum(mask_seg) if np.sum(mask_seg) > 0 else 0
            
            # Check if the mask is primarily in the depth-filtered region
            if roi_scale < 1.0:
                # Scale back to check depth values
                mask_seg_original = cv2.resize(
                    mask_seg.astype(np.uint8), 
                    (roi_w, roi_h), 
                    interpolation=cv2.INTER_NEAREST
                ).astype(bool)
                relevant_depths = roi_depth_map[mask_seg_original]
            else:
                mask_seg_original = mask_seg
                relevant_depths = roi_depth_resized[mask_seg_original]
            
            # Calculate average depth of the mask
            if len(relevant_depths) > 0:
                avg_depth = np.mean(relevant_depths)
                min_depth = np.min(relevant_depths)
                max_depth = np.max(relevant_depths)
            else:
                avg_depth = 0
                min_depth = 0
                max_depth = 0
            
            # Only keep masks that:
            # 1. Have high overlap with depth-filtered areas (>70%)
            # 2. Have average depth above threshold
            # 3. Have significant area
            depth_criteria = avg_depth > depth_threshold * 0.8  # Slightly relaxed threshold
            overlap_criteria = overlap_ratio > 0.7
            size_criteria = mask_info['area'] > 300  # Reduced minimum area
            
            if depth_criteria and overlap_criteria and size_criteria:
                # Ensure integer coordinates for bounding box
                mask_info['bbox'] = ensure_int_bbox(mask_info['bbox'])
                mask_info['avg_depth'] = avg_depth  # Store for later use
                valid_masks.append(mask_info)
                print(f"  Mask {i+1}: KEPT - area={mask_info['area']:.0f}, overlap={overlap_ratio:.2f}, avg_depth={avg_depth:.1f}")
            else:
                reason = []
                if not depth_criteria:
                    reason.append(f"depth too low ({avg_depth:.1f} <= {depth_threshold * 0.8:.1f})")
                if not overlap_criteria:
                    reason.append(f"low depth overlap ({overlap_ratio:.2f})")
                if not size_criteria:
                    reason.append(f"too small ({mask_info['area']:.0f})")
                print(f"  Mask {i+1}: FILTERED - {', '.join(reason)}")
        
        masks = valid_masks
        print(f"Filtered to {len(masks)} valid masks in depth-filtered areas")
        
        # Check if we have any valid masks
        if len(masks) == 0:
            print("Warning: No valid masks found within depth-filtered areas.")
            print("This might indicate that the depth threshold is too high.")
            
            # Try with a lower threshold as fallback
            fallback_threshold = depth_threshold * 0.6
            print(f"Trying fallback threshold: {fallback_threshold:.1f}")
            
            fallback_depth_mask = roi_depth_map > fallback_threshold
            fallback_depth_mask = np.logical_and(fallback_depth_mask, roi_mask_cropped)
            
            if np.sum(fallback_depth_mask) > 500:
                # Re-run with fallback threshold
                if roi_scale < 1.0:
                    fallback_seg = cv2.resize(fallback_depth_mask.astype(np.uint8), 
                                            (new_roi_w, new_roi_h), 
                                            interpolation=cv2.INTER_NEAREST).astype(bool)
                else:
                    fallback_seg = fallback_depth_mask
                
                fallback_mask = {
                    'segmentation': fallback_seg,
                    'bbox': ensure_int_bbox([0, 0, fallback_seg.shape[1], fallback_seg.shape[0]]),
                    'area': float(np.sum(fallback_seg)),
                    'avg_depth': np.mean(roi_depth_map[fallback_depth_mask])
                }
                masks = [fallback_mask]
                print(f"Using fallback depth-based mask with {np.sum(fallback_depth_mask)} pixels.")
            else:
                print("Fallback also failed. Using entire ROI as single mask.")
                if roi_scale < 1.0:
                    fallback_seg = depth_mask_resized
                else:
                    fallback_seg = depth_based_mask
                    
                fallback_mask = {
                    'segmentation': fallback_seg,
                    'bbox': ensure_int_bbox([0, 0, fallback_seg.shape[1], fallback_seg.shape[0]]),
                    'area': float(np.sum(fallback_seg)),
                    'avg_depth': np.mean(roi_depth_map[depth_based_mask]) if np.any(depth_based_mask) else 0
                }
                masks = [fallback_mask]
        
        # If we resized the ROI, adjust masks back to ROI size
        if roi_scale < 1.0:
            print("Rescaling masks to match ROI size")
            for mask_info in masks:
                mask = mask_info['segmentation']
                rescaled_mask = cv2.resize(
                    mask.astype(np.uint8), 
                    (roi_w, roi_h), 
                    interpolation=cv2.INTER_NEAREST
                ).astype(bool)
                mask_info['segmentation'] = rescaled_mask
                
                bbox = mask_info['bbox']
                mask_info['bbox'] = ensure_int_bbox([
                    bbox[0] / roi_scale,
                    bbox[1] / roi_scale,
                    bbox[2] / roi_scale,
                    bbox[3] / roi_scale
                ])
                
                mask_info['area'] = float(np.sum(rescaled_mask))
                
        # Transform masks back to original image coordinates
        print("Transforming masks to original image coordinates")
        for mask_info in masks:
            # Create a full-size mask
            full_mask = np.zeros((height, width), dtype=bool)
            
            # Place the ROI mask in the correct position
            roi_seg = mask_info['segmentation']
            
            # Ensure the mask is within the ROI bounds in the full image
            roi_seg_in_full = np.zeros((height, width), dtype=bool)
            roi_seg_in_full[y_start:y_end, x_start:x_end] = roi_seg
            
            # Final check: mask must be within the original ROI mask
            full_mask = np.logical_and(roi_seg_in_full, roi_mask)
            
            # Update mask info
            mask_info['segmentation'] = full_mask
            
            # Adjust bounding box to full image coordinates
            bbox = mask_info['bbox']
            mask_info['bbox'] = ensure_int_bbox([
                bbox[0] + x_start,
                bbox[1] + y_start,
                bbox[2],
                bbox[3]
            ])
            
            # Recalculate area
            mask_info['area'] = float(np.sum(full_mask))
                
    except RuntimeError as e:
        # Handle CUDA memory errors with similar approach
        if "CUDA out of memory" in str(e):
            print("CUDA out of memory error. Trying with reduced settings...")
            torch.cuda.empty_cache()
            
            # Use the same fallback approach as before but with depth filtering
            mask_generator = SAM2AutomaticMaskGenerator(
                sam_model,
                points_per_side=16,
                pred_iou_thresh=0.70,
                stability_score_thresh=0.65,
                crop_n_layers=0,
                crop_n_points_downscale_factor=4,
                min_mask_region_area=300,
                # output_mode="binary_mask"
            )
            
            masks = mask_generator.generate(roi_for_sam)
            print(f"Generated {len(masks)} masks with reduced settings")
            
            # Apply the same depth-based filtering as above
            # [Same filtering logic as in the try block]
            # ... (implement the same filtering logic)
        else:
            raise


    # 5.5. Select the two largest NON-TOUCHING masks
    
    print("\nSelecting the two largest non-touching masks")
    
    def masks_are_touching(mask1, mask2, buffer_distance=5):
        """
        Check if two masks are touching or overlapping.
        Uses morphological dilation to create a buffer zone.
        """
        # Create a buffer around mask1
        kernel = np.ones((buffer_distance*2+1, buffer_distance*2+1), np.uint8)
        mask1_buffered = binary_dilation(mask1, kernel)
        
        # Check if buffered mask1 overlaps with mask2
        overlap = np.logical_and(mask1_buffered, mask2)
        
        return np.any(overlap)
    
    def get_mask_distance(mask1, mask2):
        """
        Calculate the minimum distance between two masks in pixels.
        """
        # Get coordinates of mask pixels
        y1, x1 = np.where(mask1)
        y2, x2 = np.where(mask2)
        
        if len(y1) == 0 or len(y2) == 0:
            return float('inf')
        
        # Create coordinate arrays
        coords1 = np.column_stack((y1, x1))
        coords2 = np.column_stack((y2, x2))
        
        # Calculate minimum distance between any two points
        from scipy.spatial.distance import cdist
        distances = cdist(coords1, coords2)
        min_distance = np.min(distances)
        
        return min_distance
    
    def clean_mask(mask, min_component_size=500):
        """
        Clean a mask by removing small disconnected components and keeping only 
        the largest connected component.
        
        Args:
            mask: Binary mask (numpy array)
            min_component_size: Minimum size for components to keep (default: 500 pixels)
        
        Returns:
            Cleaned binary mask with only the largest connected component
        """
        # Convert mask to uint8 for OpenCV
        mask_uint8 = mask.astype(np.uint8)
        
        # Find all connected components
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_uint8, connectivity=8)
        
        # Skip background (label 0), start from label 1
        if num_labels <= 1:
            return mask  # No components found, return original mask
        
        # Get areas of all components (excluding background)
        component_areas = stats[1:, cv2.CC_STAT_AREA]  # Skip background at index 0
        
        # Find the largest component
        largest_component_idx = np.argmax(component_areas) + 1   
        largest_area = component_areas[largest_component_idx - 1]
        
        print(f"    Found {num_labels - 1} connected components")
        print(f"    Largest component area: {largest_area} pixels")
        print(f"    Component areas: {sorted(component_areas, reverse=True)[:5]}")  # Show top 5
        
        # Create cleaned mask with only the largest component
        cleaned_mask = (labels == largest_component_idx)
        
         
        for i in range(1, num_labels):
            component_area = stats[i, cv2.CC_STAT_AREA]
            # Keep components that are at least 20% the size of the largest component
             
            if (component_area >= min_component_size and 
                component_area >= largest_area * 0.2 and 
                i != largest_component_idx):
                cleaned_mask = np.logical_or(cleaned_mask, labels == i)
                # print(f"    Keeping additional large component with area: {component_area}")
        
        original_area = np.sum(mask)
        cleaned_area = np.sum(cleaned_mask)
        print(f"    Mask cleaning: {original_area} -> {cleaned_area} pixels ({100*cleaned_area/original_area:.1f}% retained)")
        
        return cleaned_mask
    
    # Sort masks by area (largest first)
    masks_sorted = sorted(masks, key=lambda x: x['area'], reverse=True)
    
    print(f"Available masks sorted by area:")
    for i, mask_info in enumerate(masks_sorted):
        print(f"  Mask {i+1}: area = {mask_info['area']:.0f}, avg_depth = {mask_info.get('avg_depth', 0):.1f}")
    
    # Select two largest non-touching masks
    selected_masks_info = []
    
    if len(masks_sorted) == 0:
        print("Error: No masks available for selection")
    elif len(masks_sorted) == 1:
        print("Only one mask available - cleaning and using it as the single part")
        # Clean the single mask
        cleaned_mask = clean_mask(masks_sorted[0]['segmentation'], min_component_size=300)
        masks_sorted[0]['segmentation'] = cleaned_mask
        masks_sorted[0]['area'] = float(np.sum(cleaned_mask))
        selected_masks_info = [masks_sorted[0]]
    else:
        # Clean all masks first before selection
        print("Cleaning all detected masks...")
        for i, mask_info in enumerate(masks_sorted):
            print(f"  Cleaning mask {i+1}:")
            cleaned_mask = clean_mask(mask_info['segmentation'], min_component_size=300)
            mask_info['segmentation'] = cleaned_mask
            mask_info['area'] = float(np.sum(cleaned_mask))
        
        # Re-sort after cleaning (areas might have changed)
        masks_sorted = sorted(masks_sorted, key=lambda x: x['area'], reverse=True)
        
        print(f"After cleaning - masks sorted by area:")
        for i, mask_info in enumerate(masks_sorted):
            print(f"  Mask {i+1}: area = {mask_info['area']:.0f}")
        
        # Start with the largest cleaned mask
        selected_masks_info.append(masks_sorted[0])
        print(f"Selected cleaned mask 1 (largest): area = {masks_sorted[0]['area']:.0f}")
        
        # Find the largest mask that doesn't touch the first one
        found_second = False
        min_separation_distance = 10  # Minimum pixels between masks
        
        for i, candidate_mask in enumerate(masks_sorted[1:], 1):
            candidate_seg = candidate_mask['segmentation']
            first_mask_seg = selected_masks_info[0]['segmentation']
            
            # Check if cleaned masks are touching
            if not masks_are_touching(first_mask_seg, candidate_seg, buffer_distance=3):
                # Also check minimum distance
                distance = get_mask_distance(first_mask_seg, candidate_seg)
                
                if distance >= min_separation_distance:
                    selected_masks_info.append(candidate_mask)
                    print(f"Selected cleaned mask 2: area = {candidate_mask['area']:.0f}, "
                          f"distance from mask 1 = {distance:.1f} pixels")
                    found_second = True
                    break
                else:
                    print(f"  Cleaned mask {i+1}: area = {candidate_mask['area']:.0f} - "
                          f"too close (distance = {distance:.1f} < {min_separation_distance})")
            else:
                print(f"  Cleaned mask {i+1}: area = {candidate_mask['area']:.0f} - touching/overlapping")
        
        if not found_second:
            print("Warning: Could not find a second non-touching cleaned mask")
            print("Relaxing separation criteria...")
            
            # Try with smaller separation distance
            min_separation_distance = 5
            for i, candidate_mask in enumerate(masks_sorted[1:], 1):
                candidate_seg = candidate_mask['segmentation']
                first_mask_seg = selected_masks_info[0]['segmentation']
                
                distance = get_mask_distance(first_mask_seg, candidate_seg)
                if distance >= min_separation_distance:
                    selected_masks_info.append(candidate_mask)
                    print(f"Selected cleaned mask 2 (relaxed criteria): area = {candidate_mask['area']:.0f}, "
                          f"distance = {distance:.1f} pixels")
                    found_second = True
                    break
            
            if not found_second:
                print("Still no second mask found. Using only the largest cleaned mask.")
    
    # Extract the actual masks for processing
    if len(selected_masks_info) >= 2:
        selected_masks = [selected_masks_info[0]['segmentation'], selected_masks_info[1]['segmentation']]
        print(f"\nFinal selection: 2 non-touching masks")
        print(f"  Mask 1: area = {selected_masks_info[0]['area']:.0f}")
        print(f"  Mask 2: area = {selected_masks_info[1]['area']:.0f}")
    elif len(selected_masks_info) == 1:
        selected_masks = [selected_masks_info[0]['segmentation']]
        print(f"\nFinal selection: 1 mask (no valid second mask found)")
        print(f"  Mask 1: area = {selected_masks_info[0]['area']:.0f}")
    else:
        # Fallback to ROI mask
        selected_masks = [roi_mask]
        print(f"\nFinal selection: Using ROI mask as fallback")
    
    # Visualize the selection process
    selection_vis = original_image.copy()
    
    # Draw all detected masks in light colors
    for i, mask_info in enumerate(masks_sorted):
        mask = mask_info['segmentation']
        color = [100, 100, 100]  # Light gray for non-selected
        overlay = np.zeros_like(original_image)
        overlay[mask] = color
        selection_vis = cv2.addWeighted(selection_vis, 0.9, overlay, 0.1, 0)
        
        # Draw bounding box
        bbox = ensure_int_bbox(mask_info['bbox'])
        x, y, w, h = bbox
        cv2.rectangle(selection_vis, (x, y), (x + w, y + h), color, 1)
        cv2.putText(selection_vis, f"#{i+1}", (x, y - 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    
    # Highlight selected masks in bright colors
    colors = [(0, 255, 0), (0, 0, 255)]  # Green and Blue
    for i, mask in enumerate(selected_masks):
        color = colors[i % len(colors)]
        overlay = np.zeros_like(original_image)
        overlay[mask] = color
        selection_vis = cv2.addWeighted(selection_vis, 0.7, overlay, 0.3, 0)
        
        # Find mask center for labeling
        y_indices, x_indices = np.where(mask)
        if len(y_indices) > 0:
            center_y = int(np.mean(y_indices))
            center_x = int(np.mean(x_indices))
            cv2.putText(selection_vis, f"SELECTED {i+1}", (center_x - 50, center_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    
    # cv2.imwrite(f'{args.output_dir}/mask_selection_process.jpg', selection_vis)
    
    # Create a detailed separation analysis visualization with cleaned masks
    if len(selected_masks) >= 2:
        # Before and after cleaning comparison
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Show original masks (before cleaning) vs cleaned masks
        before_vis = np.zeros_like(original_image)
        after_vis = np.zeros_like(original_image)
        
        # For comparison, we'll need to store original masks before cleaning
        # Since we already cleaned them, we'll just show the final result
        colors = [(0, 255, 0), (0, 0, 255)]  # Green and Blue
        
        for i, mask in enumerate(selected_masks):
            color = colors[i % len(colors)]
            after_vis[mask] = color
        
        # Show cleaned masks
        axes[0, 0].imshow(cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB))
        axes[0, 0].set_title('Original Image')
        axes[0, 0].axis('off')
        
        axes[0, 1].imshow(cv2.cvtColor(after_vis, cv2.COLOR_BGR2RGB))
        axes[0, 1].set_title('Cleaned Selected Masks')
        axes[0, 1].axis('off')
        
        # Separation analysis
        separation_vis = np.zeros_like(original_image)
        separation_vis[selected_masks[0]] = [0, 255, 0]  # Green
        separation_vis[selected_masks[1]] = [0, 0, 255]  # Blue
        
        # Check for overlap (should be none after cleaning)
        overlap = np.logical_and(selected_masks[0], selected_masks[1])
        separation_vis[overlap] = [255, 0, 0]  # Red for any overlap
        
        axes[1, 0].imshow(cv2.cvtColor(separation_vis, cv2.COLOR_BGR2RGB))
        axes[1, 0].set_title('Mask Separation Analysis')
        axes[1, 0].axis('off')
        
        # Distance visualization
        distance = get_mask_distance(selected_masks[0], selected_masks[1])
        
        # Create a combined visualization
        combined_vis = original_image.copy()
        overlay = np.zeros_like(original_image)
        overlay[selected_masks[0]] = [0, 255, 0]
        overlay[selected_masks[1]] = [0, 0, 255]
        combined_vis = cv2.addWeighted(combined_vis, 0.7, overlay, 0.3, 0)
        
        # # Add distance text
        # cv2.putText(combined_vis, f"Separation: {distance:.1f} pixels", 
        #             (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        # cv2.putText(combined_vis, "Cleaned Masks", 
        #             (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        
        # axes[1, 1].imshow(cv2.cvtColor(combined_vis, cv2.COLOR_BGR2RGB))
        # axes[1, 1].set_title(f'Final Result (Distance: {distance:.1f}px)')
        # axes[1, 1].axis('off')
        
        # plt.tight_layout()
        # plt.savefig(f'{args.output_dir}/cleaned_mask_analysis.png', dpi=300)
        # plt.close()
        
        # Also save the simple separation analysis
        cv2.imwrite(f'{args.output_dir}/mask_separation_analysis.jpg', separation_vis)
        print(f"Cleaned mask separation distance: {distance:.1f} pixels")
        
        # Check for any remaining scattered pixels
        for i, mask in enumerate(selected_masks):
            num_components = cv2.connectedComponents(mask.astype(np.uint8))[0] - 1  # -1 to exclude background
            print(f"Selected mask {i+1}: {num_components} connected component(s) after cleaning")
    
    # Continue with the rest of your original code from here...
    print(f"Final result: {len(selected_masks)} non-touching masks ready for processing")

    
    # 6. Process full depth map (compute once for the full image)
  
    print("\nComputing full depth map for final processing")
    torch.cuda.empty_cache()

    # Process depth at original resolution or downscale if needed
    max_depth_dim = 1024
    h, w = image_rgb.shape[:2]
    depth_scale = 1.0

    if max(h, w) > max_depth_dim:
        depth_scale = max_depth_dim / max(h, w)
        new_h, new_w = int(h * depth_scale), int(w * depth_scale)
        image_for_depth = cv2.resize(image_rgb, (new_w, new_h))
        print(f"Resized image to {new_h}x{new_w} for depth processing")
    else:
        image_for_depth = image_rgb

    # Infer depth and resize back if needed
    downscaled_depth_map = infer_depth(depth_model, image_for_depth)

    if depth_scale < 1.0:
        full_depth_map = cv2.resize(downscaled_depth_map, (width, height), interpolation=cv2.INTER_LINEAR)
    else:
        full_depth_map = downscaled_depth_map

 
    # 7. Create masked depth maps for selected non-touching parts
    
    print("Creating masked depth maps for selected parts")

    # Create masked depth maps for each selected part
    masked_depth_maps = []
    for i, mask in enumerate(selected_masks):
        # Create a new depth map with zeros everywhere except where the mask is True
        mask_depth = np.zeros_like(full_depth_map)
        mask_depth[mask] = full_depth_map[mask]
        
        # Debug: save the masked depth map to verify it's correct
        plt.figure(figsize=(10, 10))
        plt.imshow(mask_depth, cmap='inferno')
        plt.colorbar(label='Depth')
        plt.title(f"Masked Depth Map for Selected Part {i+1}")
        plt.savefig(f'{args.output_dir}/{save_name}_selected_mask{i+1}_depth.png', dpi=300)
        plt.close()
        
        # Add to our list of masked depth maps
        masked_depth_maps.append(mask_depth)
        
        # Debug: Print statistics for the masked depth map
        valid_depths = mask_depth[mask_depth > 0]
        if len(valid_depths) > 0:
            print(f"Depth map for selected part {i+1} statistics:")
            print(f"  Non-zero points: {len(valid_depths)}")
            print(f"  Min depth: {np.min(valid_depths):.4f}")
            print(f"  Max depth: {np.max(valid_depths):.4f}")
            print(f"  Mean depth: {np.mean(valid_depths):.4f}")
            print(f"  Std depth: {np.std(valid_depths):.4f}")
            print(f"  Area: {np.sum(mask)} pixels")
        else:
            print(f"Warning: No valid depths in selected mask {i+1}!")

    # Visualize the selected parts with their separation
    parts_vis = original_image.copy()
    
    # Draw each selected part with a different color
    colors = [(0, 255, 0), (0, 0, 255), (255, 0, 0)]  # Green, Blue, Red
    for i, mask in enumerate(selected_masks):
        color = colors[i % len(colors)]
        
        # Create overlay
        overlay = np.zeros_like(original_image)
        overlay[mask] = color
        parts_vis = cv2.addWeighted(parts_vis, 0.7, overlay, 0.3, 0)
        
        # Draw contours
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(parts_vis, contours, -1, color, 3)
        
        # Add label at center
        y_indices, x_indices = np.where(mask)
        if len(y_indices) > 0:
            center_y = int(np.mean(y_indices))
            center_x = int(np.mean(x_indices))
            cv2.putText(parts_vis, f"Part {i+1}", (center_x - 30, center_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
            
            # Add area information
            cv2.putText(parts_vis, f"Area: {np.sum(mask)}", 
                        (center_x - 40, center_y + 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    cv2.imwrite(f'{args.output_dir}/selected_parts.jpg', parts_vis)
    print(f"Selected {len(selected_masks)}  parts for processing")

    # 8. Divide selected non-touching parts into grids and extract normal vectors
  
    print("Dividing selected parts into grids and extracting normal vectors")
    
    def divide_into_grid(mask, depth_map, grid_size_cm=10.0):
        """
        Divide a mask into grid cells of specified size in cm.
        Returns grid cells with their positions and normal vectors.
        """
        # Get mask dimensions
        y_indices, x_indices = np.where(mask)
        if len(y_indices) == 0 or len(x_indices) == 0:
            return []

        # Calculate bounding box
        min_x, min_y = np.min(x_indices), np.min(y_indices)
        max_x, max_y = np.max(x_indices), np.max(y_indices)
        
        # Use camera parameters to estimate pixels per cm
        part_distance = np.mean(depth_map[mask & (depth_map > 0)]) / 100.0  # Convert depth to meters
        if part_distance <= 0:
            part_distance = 4.0  # Default if no valid depth data
            
        # Calculate pixels per cm based on distance
        pixels_per_cm = 19.5 / part_distance
        
        print(f"Grid calibration: Distance={part_distance:.2f}m, Using {pixels_per_cm:.2f} pixels per cm")
        
        # Calculate grid size in pixels
        grid_size_pixels = int(grid_size_cm * pixels_per_cm)
        print(f"Creating grid with cells of {grid_size_cm}cm × {grid_size_cm}cm ({grid_size_pixels}px × {grid_size_pixels}px)")
        
        # Create grid cells
        grid_cells = []
        
        # Generate grid coordinates
        for y in range(min_y, max_y, grid_size_pixels):
            for x in range(min_x, max_x, grid_size_pixels):
                # Define grid cell boundaries
                x_end = min(x + grid_size_pixels, max_x)
                y_end = min(y + grid_size_pixels, max_y)
                
                # Create grid cell mask
                grid_mask = np.zeros_like(mask, dtype=bool)
                grid_mask[y:y_end, x:x_end] = True
                
                # Intersect with the original mask
                cell_mask = grid_mask & mask
                
                # Skip if the cell doesn't contain enough mask pixels
                if np.sum(cell_mask) < 25:  # Reduced threshold for better coverage
                    continue
                
                # Get cell center
                cell_y_indices, cell_x_indices = np.where(cell_mask)
                if len(cell_y_indices) == 0:
                    continue
                    
                cell_center_x = np.mean(cell_x_indices)
                cell_center_y = np.mean(cell_y_indices)
                
                # Extract depth values for this cell
                cell_depth = depth_map[cell_mask]
                
                # Skip if no valid depth values
                if len(cell_depth) == 0 or np.all(cell_depth == 0):
                    continue
                
                # Calculate cell depth (mean of non-zero values)
                cell_depth_mean = np.mean(cell_depth[cell_depth > 0])
                
                # Add cell to the list
                grid_cells.append({
                    'mask': cell_mask,
                    'center': (cell_center_x, cell_center_y),
                    'depth': cell_depth_mean,
                    'bounds': (x, y, x_end, y_end)
                })
        
        return grid_cells
    
    def estimate_normal_vector(cell, depth_map, kernel_size=5):
        """
        Estimate the normal vector for a grid cell using the depth map.
        """
        # Get cell boundaries
        x, y, x_end, y_end = cell['bounds']
        
        # Ensure we have some margin to calculate gradients properly
        x_start = max(0, x - 1)
        y_start = max(0, y - 1)
        x_end = min(depth_map.shape[1], x_end + 1)
        y_end = min(depth_map.shape[0], y_end + 1)
        
        # Extract the depth region for this cell with margins
        depth_region = depth_map[y_start:y_end, x_start:x_end].copy()
        
        # Skip if the region is too small or has no valid depth values
        if depth_region.shape[0] < 3 or depth_region.shape[1] < 3 or not np.any(depth_region > 0):
            return np.array([0, 0, 1])  # Default normal pointing toward camera
        
        # Create a mask for valid depth values
        valid_mask = (depth_region > 0)
        if np.sum(valid_mask) < 9:  # Need at least 9 valid points
            return np.array([0, 0, 1])
        
        # Replace invalid depths with median of valid depths
        median_depth = np.median(depth_region[valid_mask])
        depth_region[~valid_mask] = median_depth
        
        # Apply Gaussian blur to reduce noise
        depth_region = cv2.GaussianBlur(depth_region, (kernel_size, kernel_size), 0)
        
        # Compute gradients with Sobel operator
        grad_y = cv2.Sobel(depth_region, cv2.CV_64F, 0, 1, ksize=3)
        grad_x = cv2.Sobel(depth_region, cv2.CV_64F, 1, 0, ksize=3)
        
        # Create normal vectors
        normals = np.dstack((-grad_x, -grad_y, np.ones_like(grad_x)))
        
        # Normalize the vectors
        norm = np.sqrt(np.sum(normals**2, axis=2))
        norm[norm == 0] = 1.0
        normals = normals / norm[:, :, np.newaxis]
        
        # Calculate robust mean normal vector from center region
        center_y_start = max(0, (normals.shape[0] // 4))
        center_y_end = min(normals.shape[0], normals.shape[0] - (normals.shape[0] // 4))
        center_x_start = max(0, (normals.shape[1] // 4))
        center_x_end = min(normals.shape[1], normals.shape[1] - (normals.shape[1] // 4))
        
        center_normals = normals[center_y_start:center_y_end, center_x_start:center_x_end]
        center_valid_mask = valid_mask[center_y_start:center_y_end, center_x_start:center_x_end]
        
        if np.any(center_valid_mask):
            mean_normal = np.mean(center_normals[center_valid_mask], axis=0)
        else:
            mean_normal = np.mean(normals[valid_mask], axis=0)
        
        # Normalize the mean normal vector
        norm = np.sqrt(np.sum(mean_normal**2))
        if norm > 0:
            mean_normal = mean_normal / norm
            if mean_normal[2] < 0:
                mean_normal = -mean_normal
        else:
            mean_normal = np.array([0, 0, 1])
        
        return mean_normal
    
    # Store grid cells and normal vectors for each selected part
    grid_cells_list = []
    normal_vectors_list = []
    
    # Process each selected non-touching part
    for i, (mask, depth_map) in enumerate(zip(selected_masks, masked_depth_maps)):
        print(f"Processing selected part {i+1} for normal vector extraction")
        
        # Divide the mask into grid cells
        grid_cells = divide_into_grid(mask, depth_map, grid_size_cm=args.grid_size)
        print(f"  Created {len(grid_cells)} grid cells")
        
        # Estimate normal vector for each grid cell
        normal_vectors = []
        for cell in grid_cells:
            normal_vector = estimate_normal_vector(cell, depth_map)
            
            # Add normal vector to the cell
            cell['normal_vector'] = normal_vector
            normal_vectors.append({
                'center': cell['center'],
                'normal': normal_vector,
                'depth': cell['depth']
            })
        
        # Store results
        grid_cells_list.append(grid_cells)
        normal_vectors_list.append(normal_vectors)
        
        # Save normal vectors to file
        np.save(f'{args.output_dir}/normal_vectors/{save_name}/selected_part{i+1}_normal_vectors.npy', 
                np.array([(v['center'][0], v['center'][1], 
                           v['normal'][0], v['normal'][1], v['normal'][2], 
                           v['depth']) for v in normal_vectors], 
                         dtype=[('center_x', 'f4'), ('center_y', 'f4'), 
                                ('normal_x', 'f4'), ('normal_y', 'f4'), ('normal_z', 'f4'),
                                ('depth', 'f4')]))


    # 9. Visualize normal vectors on selected parts
   
    print("Visualizing normal vectors on selected parts")

    def visualize_normal_vectors(image, mask, normal_vectors, title, save_path, scale_factor=40.0):
        """Create a visualization of normal vectors on an image"""
        vis_image = image.copy()
        masked_image = np.zeros_like(image)
        masked_image[mask] = image[mask]
        
        y_indices, x_indices = np.where(mask)
        if len(y_indices) == 0 or len(x_indices) == 0:
            print("Warning: Empty mask for normal vector visualization")
            return vis_image, masked_image
            
        min_x, min_y = np.min(x_indices), np.min(y_indices)
        max_x, max_y = np.max(x_indices), np.max(y_indices)
        
        width = max_x - min_x
        height = max_y - min_y
        
        grid_spacing_x = 12
        grid_spacing_y = 12
        
        print(f"Creating grid visualization with fixed spacing: {grid_spacing_x}x{grid_spacing_y} pixels")
        
        vector_lookup = {}
        for v in normal_vectors:
            x, y = int(v['center'][0]), int(v['center'][1])
            vector_lookup[(x, y)] = v
        
        vectors_drawn = 0
        vector_color = (0, 255, 0)  # Green
        
        for y in range(min_y, max_y, grid_spacing_y):
            for x in range(min_x, max_x, grid_spacing_x):
                closest_dist = float('inf')
                closest_vector = None
                
                search_radius = grid_spacing_x // 2
                for dy in range(-search_radius, search_radius+1, max(1, search_radius//4)):
                    for dx in range(-search_radius, search_radius+1, max(1, search_radius//4)):
                        test_x, test_y = x + dx, y + dy
                        
                        if 0 <= test_y < mask.shape[0] and 0 <= test_x < mask.shape[1]:
                            if mask[test_y, test_x]:
                                dist = dx*dx + dy*dy
                                if dist < closest_dist:
                                    if (test_x, test_y) in vector_lookup:
                                        closest_dist = dist
                                        closest_vector = vector_lookup[(test_x, test_y)]
                
                if closest_vector is not None:
                    center_x, center_y = int(closest_vector['center'][0]), int(closest_vector['center'][1])
                    nx, ny, nz = closest_vector['normal']
                    
                    if np.isnan(nx) or np.isnan(ny) or np.isnan(nz):
                        continue
                    
                    end_x = int(center_x + nx * scale_factor)
                    end_y = int(center_y + ny * scale_factor)
                    
                    cv2.arrowedLine(vis_image, (center_x, center_y), (end_x, end_y), vector_color, 2, tipLength=0.3)
                    cv2.arrowedLine(masked_image, (center_x, center_y), (end_x, end_y), vector_color, 2, tipLength=0.3)
                    vectors_drawn += 1
        
        print(f"Drew {vectors_drawn} normal vectors in a grid pattern")
        
        plt.figure(figsize=(15, 7))
        
        plt.subplot(121)
        plt.imshow(cv2.cvtColor(vis_image, cv2.COLOR_BGR2RGB))
        plt.title('Normal Vectors on Original Image')
        plt.axis('off')
        
        plt.subplot(122)
        plt.imshow(cv2.cvtColor(masked_image, cv2.COLOR_BGR2RGB))
        plt.title('Normal Vectors on Masked Part')
        plt.axis('off')
        
        plt.suptitle(title)
        plt.tight_layout()
        plt.savefig(save_path, dpi=300)
        plt.close()
        
        return vis_image, masked_image

    # Process each selected part
    for i, (mask, normal_vectors) in enumerate(zip(selected_masks, normal_vectors_list)):
        print(f"Creating normal vector visualization for selected part {i+1}")
        
        vis_image, masked_image = visualize_normal_vectors(
            original_image, 
            mask, 
            normal_vectors, 
            f"Normal Vectors - Selected Part {i+1}",
            f'{args.output_dir}/normal_vectors/{save_name}/selected_part{i+1}_normal_vectors_vis.png',
            scale_factor=30.0
        )
        
        cv2.imwrite(f'{args.output_dir}/normal_vectors/{save_name}/selected_part{i+1}_original_with_normals.png', vis_image)
        cv2.imwrite(f'{args.output_dir}/normal_vectors/{save_name}/selected_part{i+1}_masked_with_normals.png', masked_image)

 
    # 10. Generate point clouds with normal vectors for selected parts
  
    print("Generating point clouds with normal vectors for selected parts")

    # Create PointCloudGenerator instance
    pcg = PointCloudGenerator(save_path=f'{args.output_dir}/')

    # Process each selected part
    point_clouds = []
    for i, (mask, masked_depth_map, normal_vectors) in enumerate(zip(selected_masks, masked_depth_maps, normal_vectors_list)):
        try:
            print(f"Generating point cloud for selected part {i+1}")
            
            # Debug: save mask overlay
            mask_image = np.zeros_like(original_image)
            mask_image[mask] = original_image[mask]
            cv2.imwrite(f'{args.output_dir}/{save_name}_selected_part{i+1}_mask_overlay.png', mask_image)
            
            # Check for empty or sparse masks
            num_nonzero = np.count_nonzero(mask)
            if num_nonzero < 100:
                print(f"Warning: Selected mask {i+1} has only {num_nonzero} points - skipping point cloud generation")
                point_clouds.append(None)
                continue
                
            # Check for empty or sparse depth maps
            nonzero_depths = np.count_nonzero(masked_depth_map)
            if nonzero_depths < 100:
                print(f"Warning: Depth map for selected part {i+1} has only {nonzero_depths} non-zero points - skipping")
                point_clouds.append(None)
                continue
                
            print(f"Processing depth map with {nonzero_depths} non-zero points")
            
            # Generate point cloud
            part_pcd = pcg.create_wall_point_cloud(
                depth_map=masked_depth_map,
                image_path=args.image_path,
                wall_mask=mask,
                save_name=f"{save_name}_selected_part{i+1}",
                normal_vectors=normal_vectors
            )
            
            # Add normal vectors to the point cloud
            print(f"Adding {len(normal_vectors)} normal vectors to point cloud")

            if part_pcd is not None and len(normal_vectors) > 0:
                points = np.asarray(part_pcd.points)
                pcd_normals = np.zeros((len(points), 3))
                
                # For each point, find the closest normal vector
                for j, point in enumerate(points):
                    depth = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
                    if depth > 0:
                        # Project to image coordinates
                        fx, fy, cx, cy = 1933.5, 1950.3, 661.4, 392.0
                        x_2d = point[0] / point[2] * fx + cx
                        y_2d = point[1] / point[2] * fy + cy
                        
                        # Find closest normal vector
                        closest_dist = float('inf')
                        closest_normal = np.array([0, 0, 1])
                        
                        for nv in normal_vectors:
                            nx, ny = nv['center']
                            dist = (nx - x_2d)**2 + (ny - y_2d)**2
                            if dist < closest_dist:
                                closest_dist = dist
                                closest_normal = nv['normal']
                                
                        pcd_normals[j] = closest_normal
                
                # Add normals to point cloud
                part_pcd.normals = o3d.utility.Vector3dVector(pcd_normals)
                
                print(f"  Normal vector stats: min={np.min(pcd_normals, axis=0)}, max={np.max(pcd_normals, axis=0)}")
                print(f"  Average normal: {np.mean(pcd_normals, axis=0)}")
                
            point_clouds.append(part_pcd)
            
            # Print point cloud statistics
            if part_pcd is not None and len(np.asarray(part_pcd.points)) > 0:
                pc_points = np.asarray(part_pcd.points)
                print(f"  Generated {len(pc_points)} points")
                print(f"  X range: {np.min(pc_points[:,0]):.4f} to {np.max(pc_points[:,0]):.4f}")
                print(f"  Y range: {np.min(pc_points[:,1]):.4f} to {np.max(pc_points[:,1]):.4f}")
                print(f"  Z range: {np.min(pc_points[:,2]):.4f} to {np.max(pc_points[:,2]):.4f}")
            else:
                print("  Warning: Generated point cloud is empty or None")
                
        except Exception as e:
            print(f"Error generating point cloud for selected part {i+1}: {str(e)}")
            import traceback
            traceback.print_exc()
            point_clouds.append(None)


    # 11. Visualize point clouds with normal vectors
   
    print("Visualizing point clouds with normal vectors for selected parts")

    def visualize_point_cloud_bed_with_normals(pcd, title, save_path, show_normals=True):
        """Create a point cloud bed visualization with normal vectors"""
        if pcd is None or len(np.asarray(pcd.points)) == 0:
            print(f"Warning: Empty point cloud for {title}")
            return

        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals) if show_normals and pcd.has_normals() else None
        colors = np.asarray(pcd.colors) if pcd.has_colors() else None

        fig = plt.figure(figsize=(12, 10), dpi=150)
        ax = fig.add_subplot(111, projection='3d')

        x, y, z = points[:, 0], points[:, 1], points[:, 2]

        print(f"Point cloud info: {len(points)} points")
        print(f"X range: [{np.min(x):.3f}, {np.max(x):.3f}]")
        print(f"Y range: [{np.min(y):.3f}, {np.max(y):.3f}]")
        print(f"Z range: [{np.min(z):.3f}, {np.max(z):.3f}]")

        # Create visible point cloud
        if colors is not None and len(colors) > 0:
            ax.scatter(x, y, z, c=colors, s=2.0, alpha=0.8, label='Point Cloud')
        else:
            ax.scatter(x, y, z, c='lightgray', s=2.0, alpha=0.9, label='Point Cloud')

        # Add normal vectors
        if normals is not None:
            x_min, x_max = np.min(x), np.max(x)
            y_min, y_max = np.min(y), np.max(y)
            z_min, z_max = np.min(z), np.max(z)
            
            nx, ny = 15, 15  # Grid size for normal vectors
            x_grid = np.linspace(x_min, x_max, nx)
            y_grid = np.linspace(y_min, y_max, ny)
            
            from scipy.spatial import cKDTree
            tree = cKDTree(points)
            
            arrow_scale = 0.1 * np.max([x_max - x_min, y_max - y_min, z_max - z_min])
            
            grid_points = []
            grid_normals = []
            
            for gx in x_grid:
                for gy in y_grid:
                    query_point = np.array([gx, gy, (z_min + z_max) / 2])
                    distances, indices = tree.query(query_point)
                    
                    nearest_idx = indices
                    nearest_point = points[nearest_idx]
                    nearest_normal = normals[nearest_idx]
                    
                    if np.isnan(nearest_normal).any() or np.linalg.norm(nearest_normal) == 0:
                        continue
                    
                    grid_points.append(nearest_point)
                    grid_normals.append(nearest_normal)
            
            if grid_points:
                grid_points = np.array(grid_points)
                grid_normals = np.array(grid_normals)
                
                ax.quiver(
                    grid_points[:, 0], grid_points[:, 1], grid_points[:, 2],
                    grid_normals[:, 0], grid_normals[:, 1], grid_normals[:, 2],
                    color='green',
                    length=arrow_scale,
                    normalize=True,
                    linewidth=1.5,
                    arrow_length_ratio=0.3,
                    alpha=0.8,
                    label='Normal Vectors'
                )
                
                print(f"Displayed {len(grid_points)} normal vectors on point cloud bed")

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')  
        ax.set_zlabel('Z (m)')
        ax.set_title(title)
        ax.legend()
        ax.view_init(elev=30, azim=45)
        ax.set_box_aspect([np.ptp(x), np.ptp(y), np.ptp(z)])
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"Point cloud bed visualization saved to {save_path}")

    # Visualize point clouds for selected parts
    for i, pcd in enumerate(point_clouds):
        if pcd is not None:
            print(f"Creating point cloud bed visualization for selected part {i+1}")
            
            visualize_point_cloud_bed_with_normals(
                pcd,
                f"Point Cloud Bed with Normals - Selected Part {i+1}",
                f'{args.output_dir}/normal_vectors/{save_name}/selected_part{i+1}_point_cloud_bed.png'
            )

   
    # 12. Create BTFace representations for selected parts
 
    print("Creating BTFace representations for selected parts")
    
    class BTFace:
        def __init__(self, id, surface_type, normal_vector, center_point, boundary_points, cv_value):
            self.id = id
            self.surface_type = surface_type
            self.normal_vector = normal_vector
            self.center_point = center_point
            self.boundary_points = boundary_points
            self.cv_value = cv_value
            self.btFaceMatrix = None
            
        def is_curved(self):
            return self.surface_type == 'curved'
    
    class BTFaceMatrix:
        def __init__(self, id, parent_face, segments):
            self.id = id
            self.parent_face = parent_face
            self.segments = segments
            self.cv_value = parent_face.cv_value
    
    # Create BTFace objects for each selected part
    bt_faces = []
    bt_face_matrices = []
    
    for i, (mask, normal_vectors, grid_cells) in enumerate(zip(selected_masks, normal_vectors_list, grid_cells_list)):
        y_indices, x_indices = np.where(mask)
        if len(y_indices) == 0 or len(x_indices) == 0:
            continue
            
        # Calculate mask centroid
        center_x = np.mean(x_indices)
        center_y = np.mean(y_indices)
        
        # Calculate average depth for center of volume
        mask_depths = full_depth_map[mask]
        if len(mask_depths) > 0:
            cv_value = np.mean(mask_depths)
        else:
            cv_value = 0
        
        # Calculate average normal vector
        if len(normal_vectors) > 0:
            avg_normal = np.mean([v['normal'] for v in normal_vectors], axis=0)
            norm = np.sqrt(np.sum(avg_normal**2))
            if norm > 0:
                avg_normal = avg_normal / norm
        else:
            avg_normal = np.array([0, 0, 1])
        
        # Create boundary points
        min_x, min_y = np.min(x_indices), np.min(y_indices)
        max_x, max_y = np.max(x_indices), np.max(y_indices)
        boundary_points = [
            np.array([min_x, min_y, cv_value]),
            np.array([max_x, min_y, cv_value]),
            np.array([max_x, max_y, cv_value]),
            np.array([min_x, max_y, cv_value])
        ]
        
        # Determine if surface is curved
        if len(normal_vectors) > 1:
            normal_variance = np.var([v['normal'] for v in normal_vectors], axis=0)
            total_variance = np.sum(normal_variance)
            is_curved = total_variance > 0.01
        else:
            is_curved = False
            
        # Create main BTFace
        bt_face = BTFace(
            id=f"BTF-Selected-{i+1}",
            surface_type='curved' if is_curved else 'planar',
            normal_vector=avg_normal,
            center_point=np.array([center_x, center_y, cv_value]),
            boundary_points=boundary_points,
            cv_value=cv_value
        )
        
        bt_faces.append(bt_face)
        
        # Create BTFaceMatrix for grid segments
        segments = []
        for j, cell in enumerate(grid_cells):
            segment = BTFace(
                id=f"BTF-Selected-{i+1}-seg-{j}",
                surface_type='planar',
                normal_vector=cell['normal_vector'],
                center_point=np.array([cell['center'][0], cell['center'][1], cell['depth']]),
                boundary_points=[],
                cv_value=cell['depth']
            )
            segments.append(segment)
        
        bt_face_matrix = BTFaceMatrix(
            id=f"BTF-Matrix-Selected-{i+1}",
            parent_face=bt_face,
            segments=segments
        )
        
        bt_face.btFaceMatrix = bt_face_matrix
        bt_face_matrices.append(bt_face_matrix)
    
    # Save BTFace data for selected parts
    print("Saving BTFace data for selected parts")
    for i, (bt_face, bt_face_matrix) in enumerate(zip(bt_faces, bt_face_matrices)):
        bt_face_data = {
            'id': bt_face.id,
            'surface_type': bt_face.surface_type,
            'normal_vector': bt_face.normal_vector.tolist(),
            'center_point': bt_face.center_point.tolist(),
            'boundary_points': [p.tolist() for p in bt_face.boundary_points],
            'cv_value': float(bt_face.cv_value),
            'segment_count': len(bt_face_matrix.segments),
            'segments': [
                {
                    'id': seg.id,
                    'normal_vector': seg.normal_vector.tolist(),
                    'center_point': seg.center_point.tolist(),
                    'cv_value': float(seg.cv_value)
                }
                for seg in bt_face_matrix.segments
            ]
        }
        
        import json
        with open(f'{args.output_dir}/normal_vectors/{save_name}/selected_part{i+1}_bt_face_data.json', 'w') as f:
            json.dump(bt_face_data, f, indent=2)
    
    print("Process completed successfully for selected non-touching parts")
    print(f"Results saved to {args.output_dir}/")
    print(f"Processed {len(selected_masks)} selected non-touching parts")

if __name__ == "__main__":
    main()



    