from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_opencv_tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='parallels@example.com',
    description='ROS 2 OpenCV optical flow package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'optical_flow = my_opencv_tutorials.optical_flow:main',
            'img_pub = my_opencv_tutorials.img_pub:main',
        ],
    },
)
