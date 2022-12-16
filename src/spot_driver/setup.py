from setuptools import setup
import os
from glob import glob

package_name = 'spot_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='achille.morenville',
    maintainer_email='achille.morenville@uclouvain.be',
    description='Driver for Spot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spot_odometry_pub = spot_driver.spot_odometry_pub:main'
        ],
    },
)
