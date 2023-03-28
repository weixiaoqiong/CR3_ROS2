from setuptools import setup
import os
from glob import glob

package_name = 'object_world_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share',package_name,'launch'), glob('launch/*_launch.py')),
        (os.path.join('share',package_name,'config'), glob('config/*.yaml')),     
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
            'object_world_pub=object_world_pub.object_world_pub:main',
            'cr3_object_world_pub_test=object_world_pub.cr3_object_world_pub_test:main',
            'test=object_world_pub.test_tmp:main',
            'pixel_fake_pub = object_world_pub.pixel_fake_pub:main',
            'pixel_tf_fake_pub = object_world_pub.pixel_tf_fake_pub:main',            
            'cam_info_fake_pub = object_world_pub.cam_info_fake_pub:main',
            # 'object_world_pub_new=object_world_pub.object_world_pub_new:main',
            'object_world_pub_bunker=object_world_pub.object_world_pub_bunker:main',
            'object_world_pub_bunker_gazebo=object_world_pub.object_world_pub_bunker_gazebo:main',
            'object_world_pub_bunker_gazebo3=object_world_pub.object_world_pub_bunker_gazebo3:main',
            'object_world_pub_bunker_gazebo4=object_world_pub.object_world_pub_bunker_gazebo4:main',
            'object_world_pub_bunker_gazebo5=object_world_pub.object_world_pub_bunker_gazebo5:main',
            'object_world_pub_bunker_gazebo6=object_world_pub.object_world_pub_bunker_gazebo6:main',
            'image_undistor=object_world_pub.image_undistor:main'
        ],
    },
)
