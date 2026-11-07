from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'revd_lidar_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Safwan',
    maintainer_email='safwan.shaikh@gmail.com',
    description='Sensor models, odometry evaluation and simulation cleanup for revD_lidar.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'lidar_sim = revd_lidar_tools.lidar_sim:main',
        'tof_ray_markers = revd_lidar_tools.tof_ray_markers:main',
        'tof_rows = revd_lidar_tools.tof_rows:main',
        'tof_sim = revd_lidar_tools.tof_sim:main',
        'stop_simulation = revd_lidar_tools.stop_simulation:main',
        'odom_eval = revd_lidar_tools.odom_eval:main',
        'odom_eval_plot = revd_lidar_tools.odom_eval_plot:main',
        'imu_bias = revd_lidar_tools.imu_bias:main',
        'safety_governor = revd_lidar_tools.safety_governor:main',
    ]},
)
