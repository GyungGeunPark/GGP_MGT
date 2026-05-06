#!/usr/bin/env python3
"""
nimg_v3 Setup Script
"""

from setuptools import setup, find_packages

setup(
    name='nimg_v3',
    version='3.0.0',
    description='FoundationPose-based 6DoF Pose Estimation System',
    author='FurSys AI Team',
    packages=find_packages(),
    python_requires='>=3.8',
    install_requires=[
        'numpy>=1.20.0',
        'scipy>=1.7.0',
        'opencv-python>=4.5.0',
        'filterpy>=1.4.5',
        'pyyaml>=5.4.0',
        'pandas>=1.3.0',
    ],
    extras_require={
        'full': [
            'torch>=1.10.0',
            'trimesh>=3.9.0',
            'scikit-image>=0.18.0',
            'ultralytics>=8.0.0',
        ],
        'dev': [
            'pytest>=6.0.0',
            'pytest-cov>=2.0.0',
        ],
    },
    entry_points={
        'console_scripts': [
            'nimg_v3_train=scripts.train_neural_field:main',
            'nimg_v3_measure=scripts.run_measurement:main',
        ],
    },
)
