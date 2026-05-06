from setuptools import find_packages, setup

package_name = 'calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam_ws',
    maintainer_email='maint@example.com',
    description='Dual-board two-stage calibration pipeline.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calib_stage1 = calibration.stage1.stage1_runner:main_cli',
            'calib_stage2 = calibration.stage2.stage2_runner:main_cli',
            'calib_generate_board_a = calibration.tools.generate_board_a_r2:main',
            'calib_generate_board_b = calibration.tools.generate_board_b_r3:main',
            'calib_verify_install  = calibration.tools.verify_install:main',
        ],
    },
)
