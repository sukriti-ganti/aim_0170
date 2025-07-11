from setuptools import setup
import os
from glob import glob

package_name = 'b3rb_ros_aim_india'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line to install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yeml]'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mayank',
    maintainer_email='mayankmahajan.x@nxp.com',
    description='MR-B3RB Line Follower',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'explore = b3rb_ros_aim_india.b3rb_ros_warehouse:main',
                'visualize = b3rb_ros_aim_india.b3rb_ros_draw_map:main',
                'remover = b3rb_ros_aim_india.b3rb_ros_model_remove:main',
                # 'detect = b3rb_ros_aim_india.b3rb_ros_object_recog:main',
        ],
    },
)
