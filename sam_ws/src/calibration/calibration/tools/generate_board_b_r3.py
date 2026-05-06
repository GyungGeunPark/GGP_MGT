"""Generate Board B r3 printable image."""
from __future__ import annotations
import argparse
from ._board_generator import generate_charuco_image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--yaml', default='/root/sam_ws/config/board_b_r3_params.yaml')
    parser.add_argument('--out', default='/root/sam_ws/calibration_results/board_b_r3_print.png')
    parser.add_argument('--dpi', type=int, default=150)
    args = parser.parse_args()
    path = generate_charuco_image(args.yaml, args.out, dpi=args.dpi)
    print(f"Board B image written: {path}")


if __name__ == '__main__':
    main()
