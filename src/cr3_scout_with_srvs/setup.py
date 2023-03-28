from setuptools import setup
import os
from glob import glob

package_name = 'cr3_scout_with_srvs'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        #(os.path.join('share',package_name,'launch'), glob('launch/*_launch.py')),
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
            'scout = cr3_scout_with_srvs.scout:main',
            'client = cr3_scout_with_srvs.cr3_service_clients:main',
            'scout_with_zoom_camera = cr3_scout_with_srvs.scout_with_zoom_camera:main',
            'camera_intrin_info = cr3_scout_with_srvs.camera_intrin_info:main',
            'scout_with_zoom_camera_horiz = cr3_scout_with_srvs.scout_with_zoom_camera_horiz:main',
            #'cam_info_fake_pub = cr3_scout_with_srvs.cam_info_fake_pub:main'
        ],
    },
)
