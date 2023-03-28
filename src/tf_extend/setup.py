from setuptools import setup
import os
from glob import glob

package_name = 'tf_extend'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share',package_name,'launch'), glob('launch/*_launch.py')),  
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qiuyuezuo',
    maintainer_email='qiuyuezuo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Link6_to_camera=tf_extend.Link6_to_camera:main',
            'world_to_cr3_dummy=tf_extend.world_to_cr3_dummy:main',
            'world_to_camera_test=tf_extend.world_to_camera_test:main',
            'world_to_camera_pub=tf_extend.world_to_camera_pub:main',
            'bunker_to_camera=tf_extend.bunker_to_camera:main',
            'world_to_camera_bunker_pub=tf_extend.world_to_camera_bunker_pub:main'
        ],
    },
)
