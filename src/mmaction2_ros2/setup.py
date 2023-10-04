from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mmaction2_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='takane',
    maintainer_email='takanen20010829kota@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"hello_node = {package_name}.hello_node:main",
            f"mmaction2_node = {package_name}.mmaction2_node:main",
        ],
    },
)
