#!/usr/bin/env python3
import torch
import cv2
from depth_anything_v2.dpt import DepthAnythingV2
from depth_anything_v2.util.transform import Resize, NormalizeImage, PrepareForNet
import torchvision.transforms as transforms
import numpy as np

def load_depth_model(encoder, device='cuda'):
    """Depth Anything 모델을 로드합니다."""
   
    DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
    
    model_configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
        'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
    }
    
    depth_anything = DepthAnythingV2(**model_configs['vitb'])
    depth_anything.load_state_dict(torch.load(encoder, map_location='cpu'))
    depth_anything = depth_anything.to(DEVICE).eval()    
    # 가중치 로드

    transform = transforms.Compose([
        Resize(
            width=518,
            height=518,
            resize_target=False,
            keep_aspect_ratio=True,
            ensure_multiple_of=14,
            resize_method="minimal",
            image_interpolation_method=cv2.INTER_CUBIC,
        ),
        NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        PrepareForNet(),
    ])
    
    depth_anything.transform = transform
    return depth_anything

def infer_depth(model, image):
    """이미지로부터 깊이 맵을 생성합니다."""
    with torch.no_grad():
        depth = model.infer_image(image, input_size=518)
        depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
        depth = depth.astype(np.uint8)
    return depth 
