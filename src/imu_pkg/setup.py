from setuptools import find_packages, setup
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'imu_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soroush',
    maintainer_email='soroush.kabiri.92@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'orientation_publisher = imu_pkg.orientation_publisher:main',
        'madgwick_to_yaw = imu_pkg.madgwick_to_yaw:main',
        'catch_mag_data = imu_pkg.catch_mag_data:main',

        ],
    },
)
