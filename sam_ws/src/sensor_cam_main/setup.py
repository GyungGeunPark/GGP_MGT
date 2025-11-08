from setuptools import setup
import os
from glob import glob

package_name = 'sensor_cam_main'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    # 패키지 안에 포함해야 할 데이터(모델 파일 등)
    data_files=[
        # 아래 두 줄은 ROS2 패키지 식별에 필요한 설정
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 실제 모델 파일을 설치할 위치를 지정
        # 여기서는 lib/<패키지이름>/models 에 넣도록 했다.
        (
            'share/' + package_name + '/models', glob('models/*.pth')
        ),
        (
        	'share/' + package_name + '/calibration', glob('calibration/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jyp',
    maintainer_email='jyp7781@ff00ff.kr',
    description='Some description',
    license='License declaration',
    entry_points={
        'console_scripts': [
            'detection = sensor_cam_main.detection:main',
        ],
    },
)

