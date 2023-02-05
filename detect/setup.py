import os
from glob import glob
from setuptools import setup

package_name = 'detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
        glob(os.path.join('launch', '*_launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='enpit',
    maintainer_email='io1007@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = detect.detection:main',
            'detector2 = detect.detection2:main',
            'camera_server = detect.camera_server:main',
            'setup_stereocamera = detect.setup_stereocamera:main',
        ],
    },
)
