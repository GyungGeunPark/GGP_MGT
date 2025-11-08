#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
from ament_index_python.packages import get_package_share_directory
import os
import gzip
import yaml
import sys
import argparse

# Segment Anything related
from segment_anything import sam_model_registry, SamPredictor
from segment_anything import SamAutomaticMaskGenerator

# Depth Anything v2 related
from depth_utils import load_depth_model, infer_depth
from d2p import PointCloudGenerator
from depth_anything_v2.dpt import DepthAnythingV2

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Process an image to detect equipment parts and generate 3D point clouds.')
    parser.add_argument('image_path', type=str, help='Path to the input image file')
    parser.add_argument('--output_dir', type=str, default='./detection', help='Directory to save output files')
    parser.add_argument('--device', type=str, default='cuda' if torch.cuda.is_available() else 'cpu', 
                        help='Device to use for computation (cuda/cpu)')
    
    args = parser.parse_args()
    
    # Validate input image path
    if not os.path.exists(args.image_path):
        print(f"Error: Input image not found at path: {args.image_path}")
        sys.exit(1)
    
    # Extract file name without extension to use as save_name
    input_name = os.path.splitext(os.path.basename(args.image_path))[0]
    save_name = input_name
    
    # (변경 부분) 결과물을 저장할 폴더 생성: output_dir/save_name
    result_folder = os.path.join(args.output_dir, save_name)
    os.makedirs(result_folder, exist_ok=True)

    # -----------------------------
    # 1. Load models
    # -----------------------------
    device = torch.device(args.device)
    print(f"Using device: {device}")
    
    share_dir = get_package_share_directory('sensor_cam_main')

    # (1) SAM model loading
    print("Loading SAM model")
    sam_checkpoint = os.path.join(share_dir, "models", "sam_vit_b_01ec64.pth")
    sam_type = "vit_b"  # Model type: vit_h, vit_l, vit_b etc.
    sam_model = sam_model_registry[sam_type](checkpoint=sam_checkpoint)
    sam_model.to(device)
    predictor = SamPredictor(sam_model)

    # (2) Depth Anything v2 model loading
    print("Loading Depth Anything v2 model")
    depth_checkpoint = os.path.join(share_dir, "models", "depth_anything_v2_vitb.pth")
    depth_model = load_depth_model(depth_checkpoint, device=device)

    # -----------------------------
    # 2. Load image
    # -----------------------------
    print(f"Loading image: {args.image_path}")
    original_image = cv2.imread(args.image_path)
    if original_image is None:
        print(f"Error: Could not load image at {args.image_path}")
        sys.exit(1)
        
    # Save original dimensions for final visualization
    original_height, original_width = original_image.shape[:2]

    # BGR to RGB conversion (필요하다면 적용)
    if original_image.ndim == 3 and original_image.shape[2] == 3:
        image_rgb = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
    else:
        # Grayscale이거나 채널 수가 다를 경우에는 그대로 사용
        image_rgb = original_image

    print(f"Original image dimensions: {original_width}x{original_height}")

    # -----------------------------
    # 3. Resize for memory concerns (512×512)
    # -----------------------------
    max_dim = 512
    oh, ow = original_height, original_width
    scale = 1.0
    if max(oh, ow) > max_dim:
        scale = max_dim / float(max(oh, ow))
        new_h, new_w = int(oh * scale), int(ow * scale)
        image_for_sam = cv2.resize(image_rgb, (new_w, new_h))
        print(f"Resized image from {ow}x{oh} to {new_w}x{new_h} for SAM/Depth processing")
    else:
        image_for_sam = image_rgb

    # 이제부터의 height, width는 "리사이즈된" 이미지의 크기를 의미
    height, width = image_for_sam.shape[:2]
    print(f"Working resolution: {width}x{height}")

    # -----------------------------
    # 4. Run SAM + Depth on the resized image
    # -----------------------------
    torch.cuda.empty_cache()

    # Set SAM predictor
    predictor.set_image(image_for_sam)

    # Depth inference
    image_for_depth = image_for_sam
    downscaled_depth_map = infer_depth(depth_model, image_for_depth)
    full_depth_map = downscaled_depth_map  # (height, width) 크기의 Depth

    print("Generating SAM masks")
    mask_generator = SamAutomaticMaskGenerator(
        sam_model,
        points_per_side=32,
        pred_iou_thresh=0.88,
        stability_score_thresh=0.92,
        crop_n_layers=1,
        crop_n_points_downscale_factor=2,
        min_mask_region_area=2000,
        output_mode="binary_mask",
        box_nms_thresh=0.7,
        point_grids=None
    )

    try:
        masks = mask_generator.generate(image_for_sam)
        print(f"Generated {len(masks)} masks")
    except RuntimeError as e:
        if "CUDA out of memory" in str(e):
            print("CUDA out of memory error. Trying with even smaller image and simpler settings...")
            torch.cuda.empty_cache()
            
            # 더 작은 해상도로 재시도
            max_dim = 384
            new_h, new_w = int(oh * (max_dim / float(max(oh, ow)))), int(ow * (max_dim / float(max(oh, ow))))
            image_for_sam = cv2.resize(image_rgb, (new_w, new_h))
            predictor.set_image(image_for_sam)
            mask_generator = SamAutomaticMaskGenerator(
                sam_model,
                points_per_side=16,
                pred_iou_thresh=0.88,
                stability_score_thresh=0.92,
                crop_n_layers=0,
                crop_n_points_downscale_factor=4,
                min_mask_region_area=2000,
                output_mode="binary_mask"
            )
            masks = mask_generator.generate(image_for_sam)
            print(f"Generated {len(masks)} masks with reduced settings")
            
            # 다시 height, width 갱신
            height, width = image_for_sam.shape[:2]
            full_depth_map = infer_depth(depth_model, image_for_sam)
        else:
            raise

    # -----------------------------
    # 5. Identify equipment parts (in the resized domain)
    # -----------------------------
    def is_equipment_part(mask, image_center, depth_map):
        y_indices, x_indices = np.where(mask)
        if len(y_indices) == 0 or len(x_indices) == 0:
            return False
        
        bbox = [np.min(x_indices), np.min(y_indices), 
                np.max(x_indices) - np.min(x_indices), 
                np.max(y_indices) - np.min(y_indices)]
        
        mask_center_x = np.mean(x_indices)
        mask_center_y = np.mean(y_indices)
        
        distance_to_center = np.sqrt((image_center[0] - mask_center_x)**2 + 
                                     (image_center[1] - mask_center_y)**2)
        
        area = len(x_indices)
        
        avg_depth = 0
        if depth_map is not None:
            mask_depths = depth_map[mask]
            if len(mask_depths) > 0:
                avg_depth = np.mean(mask_depths)
        
        edge_margin = min(width, height) * 0.05
        not_at_edge = (bbox[0] > edge_margin and 
                       bbox[1] > edge_margin and 
                       bbox[0] + bbox[2] < width - edge_margin and 
                       bbox[1] + bbox[3] < height - edge_margin)
        
        max_center_distance = min(width, height) * 0.5
        is_central = distance_to_center < max_center_distance
        
        is_significant = area > 2000
        
        return not_at_edge and is_central and is_significant

    debug_image = image_for_sam.copy()
    parts_visualization = np.zeros_like(image_for_sam)
    image_center = (width // 2, height // 2)
    equipment_parts = []

    print("Identifying equipment parts")
    for i, mask_info in enumerate(masks):
        seg_mask = mask_info['segmentation']
        if is_equipment_part(seg_mask, image_center, full_depth_map):
            mask_info['mask_id'] = i + 1
            equipment_parts.append(mask_info)
            
            color = np.random.randint(0, 255, size=3).tolist()
            parts_visualization[seg_mask] = color
            
            bbox = mask_info['bbox']
            x, y, w_box, h_box = bbox
            
            cv2.rectangle(debug_image, (x, y), (x + w_box, y + h_box), color, 2)
            cv2.putText(debug_image, 
                        f"#{i+1} Area:{mask_info['area']:.0f}", 
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        color,
                        2)

    print(f"Identified {len(equipment_parts)} equipment parts")

    equipment_parts.sort(key=lambda x: x['area'], reverse=True)

    # (변경 부분) 모든 결과 파일을 result_folder에 저장
    cv2.imwrite(os.path.join(result_folder, 'equipment_parts_debug_512.jpg'), debug_image)
    cv2.imwrite(os.path.join(result_folder, 'equipment_parts_visualization_512.jpg'), parts_visualization)

    # -----------------------------
    # 6. Select the largest part (still in 512×512 domain)
    # -----------------------------
    print("Selecting the largest equipment part")
    if not equipment_parts:
        print("Warning: No equipment parts detected in the resized image")
        selected_mask = np.ones((height, width), dtype=bool)
    else:
        largest_part = equipment_parts[0]
        selected_mask = largest_part['segmentation']
        print(f"Selected part #{largest_part['mask_id']} with area {largest_part['area']:.0f}")

        if full_depth_map is not None:
            masked_depth = full_depth_map * selected_mask
            valid_depths = masked_depth[masked_depth > 0]
            
            if len(valid_depths) > 0:
                mean_depth = np.mean(valid_depths)
                std_depth = np.std(valid_depths)
                
                depth_threshold = 1.0 * std_depth
                refined_mask = np.abs(full_depth_map - mean_depth) < depth_threshold
                refined_mask = refined_mask & selected_mask
                
                if np.sum(refined_mask) > 0.7 * np.sum(selected_mask):
                    selected_mask = refined_mask
                    print("Used depth-refined mask")

    # -----------------------------
    # 7. Create masked depth map (512×512)
    # -----------------------------
    print("Creating masked depth map")
    masked_depth_map = np.zeros_like(full_depth_map)
    masked_depth_map[selected_mask] = full_depth_map[selected_mask]

    # -----------------------------
    # 8. Visualization
    #    - (A) 512×512 디버그/결과
    #    - (B) 최종 원본 해상도에만 마스크를 리사이즈해서 겹치기
    # -----------------------------

    plt.figure(figsize=(20, 10))

    # 8A-1. 축소된 이미지에서의 선택 파트
    plt.subplot(221)
    plt.title("Selected Largest Part (512×512)")
    outlined_512 = image_for_sam.copy()
    contours_512, _ = cv2.findContours(selected_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(outlined_512, contours_512, -1, (0, 255, 0), 3)
    plt.imshow(outlined_512)
    plt.axis('off')

    # 8A-2. 선택 마스크 (512×512)
    plt.subplot(222)
    plt.title("Part Mask (512×512)")
    plt.imshow(selected_mask, cmap='gray')
    plt.axis('off')

    # 8A-3. 전체 Depth Map (512×512)
    plt.subplot(223)
    plt.title("Full Depth Map (512×512)")
    depth_vis_512 = (full_depth_map - full_depth_map.min()) / (full_depth_map.max() - full_depth_map.min() + 1e-8)
    plt.imshow(depth_vis_512, cmap='inferno')
    plt.axis('off')

    # 8A-4. 선택 마스크 적용된 Depth Map (512×512)
    plt.subplot(224)
    plt.title("Masked Part Depth Map (512×512)")
    masked_depth_vis_512 = np.zeros_like(depth_vis_512)
    masked_depth_vis_512[selected_mask] = depth_vis_512[selected_mask]
    plt.imshow(masked_depth_vis_512, cmap='inferno')
    plt.axis('off')

    plt.tight_layout()
    plt.savefig(os.path.join(result_folder, f'{save_name}_analysis_512.png'), bbox_inches='tight', dpi=300)
    plt.close()

    # 8B. 원본 해상도로 마스크 리사이즈 후, 원본 이미지에 Contour
    resized_selected_mask = cv2.resize(
        selected_mask.astype(np.uint8),
        (original_width, original_height),
        interpolation=cv2.INTER_NEAREST
    ).astype(bool)

    outlined_image_original = original_image.copy()
    contours_orig, _ = cv2.findContours(resized_selected_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(outlined_image_original, contours_orig, -1, (0, 255, 0), 3)

    cv2.imwrite(os.path.join(result_folder, f'{save_name}_outline_original.jpg'), outlined_image_original)

    # -----------------------------
    # 9. Generate point cloud (512×512)
    # -----------------------------
    print("Generating point cloud")

    # (변경 부분) PointCloudGenerator에 result_folder를 넘겨준다.
    pcg = PointCloudGenerator(save_path=result_folder)

    try:
        wall_pcd = pcg.create_wall_point_cloud(
            depth_map=masked_depth_map,
            image_path=args.image_path,   # 원본 이미지 경로
            wall_mask=selected_mask,
            save_name=save_name
        )
        
        print("Point cloud generated successfully")

        # 10. 3D Visualization in 512 domain
        print("Generating 3D visualization")
        depth_vis_512 = (full_depth_map - full_depth_map.min()) / (full_depth_map.max() - full_depth_map.min() + 1e-8)
        masked_depth_vis_512 = np.zeros_like(depth_vis_512)
        masked_depth_vis_512[selected_mask] = depth_vis_512[selected_mask]

        pc_points = np.asarray(wall_pcd.points)
        if len(pc_points) == 0:
            print("Warning: Point cloud is empty. Check depth map and mask")
        else:
            x, y, z = pc_points[:, 0], pc_points[:, 1], pc_points[:, 2]

            fig = plt.figure(figsize=(15, 5))
            try:
                ax1 = fig.add_subplot(131)
                ax1.imshow(outlined_512)
                ax1.set_title("Selected Part (512)")
                ax1.axis("off")

                ax2 = fig.add_subplot(132)
                im = ax2.imshow(masked_depth_vis_512, cmap="inferno")
                ax2.set_title("Masked Depth Map (512)")
                ax2.axis("off")
                cbar = plt.colorbar(im, ax=ax2)
                cbar.set_label("Depth")

                ax3 = fig.add_subplot(133, projection='3d')
                sc = ax3.scatter(x, y, z, c=z, cmap="inferno", s=2)
                ax3.set_title("Point Cloud (Wall)")
                ax3.set_xlabel("X (m)")
                ax3.set_ylabel("Y (m)")
                ax3.set_zlabel("Z (m)")
            except Exception as e:
                print(f"Warning: 3D visualization failed - {str(e)}")
                try:
                    ax3 = fig.add_subplot(133)
                    sc = ax3.scatter(x, y, c=z, cmap="inferno", s=2)
                    ax3.set_title("Point Cloud (2D Projection)")
                    ax3.set_xlabel("X (m)")
                    ax3.set_ylabel("Y (m)")
                    plt.colorbar(sc, ax=ax3, label="Z (m)")
                except Exception as e2:
                    print(f"Warning: Fallback visualization also failed - {str(e2)}")

            plt.tight_layout()
            plt.savefig(os.path.join(result_folder, f'{save_name}_final_visualization_3d.png'), bbox_inches='tight', dpi=300)
            plt.close()
            
            print("Visualization generated successfully!")
        
    except Exception as e:
        print(f"Error generating point cloud: {str(e)}")
        print("Process completed with errors")
        return

    print("Process completed successfully")
    print(f"Results saved to {result_folder}/")


if __name__ == "__main__":
    main()