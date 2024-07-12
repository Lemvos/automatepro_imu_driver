import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'automatepro_imu_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), 
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'adafruit_bno08x'
        ],
    zip_safe=True,
    maintainer='Balachandra',
    maintainer_email='balachandra.bhat@lemvos.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_driver = automatepro_imu_driver.imu_driver:main',
        ],
    },
)
