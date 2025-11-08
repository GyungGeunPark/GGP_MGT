from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'realsense_robot_control'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 패키지 인덱스 및 매니페스트 파일 (기존과 동일)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch 파일 (기존과 동일)
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*launch.[pxy][yma]*')),
        
        # Config 파일 (기존과 동일)
        (os.path.join('share', package_name, 'config'), 
            glob('config/*')),
        
        # Test data 디렉토리 (기존과 동일)
        (os.path.join('share', package_name, 'test_data'),
            glob('test_data/*.json')),

        # ---▼▼▼▼▼ [수정/개선된 부분] ▼▼▼▼▼---
        
        # scripts 폴더 안의 모든 파이썬 스크립트(.py)를 설치합니다.
        # 이렇게 하면 sd.py, d2p.py 등을 일일이 명시할 필요가 없습니다.
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.py')),
            
        # pth 폴더 안의 모든 모델 파일(.pth)을 설치합니다.
        # 기존 코드가 이미 올바르게 작성되어 있었습니다.
        (os.path.join('share', package_name, 'pth'),
            glob('pth/*.*')),
        
        # ---▲▲▲▲▲ [수정/개선된 부분] ▲▲▲▲▲---
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='RealSense D455 vision-based robot control system with integrated surface detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realsense_viewer = realsense_robot_control.realsense_viewer:main',
            'realsense_viewer_v4 = realsense_robot_control.realsense_viewerv2:main',
            'robot_controller = realsense_robot_control.robot_controller:main',
            'cobot_interface = gt3_rainbow.main:main',
            
            # ---▼▼▼▼▼ [수정된 부분] ▼▼▼▼▼---
            # 'surface_detector = scripts.sd:main' 라인은 'scripts'가 파이썬 패키지가 아니므로
            # 정상적으로 동작하지 않습니다. realsense_viewerv2.py에서 직접 스크립트를
            # 실행하므로 이 entry_point는 필요하지 않습니다. 주석 처리하거나 삭제합니다.
            # 'surface_detector = scripts.sd:main',
            # ---▲▲▲▲▲ [수정된 부분] ▲▲▲▲▲---
        ],
    },
)
