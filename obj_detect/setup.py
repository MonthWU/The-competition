from setuptools import find_packages, setup
from glob import glob
import os
import warnings
warnings.filterwarnings("ignore")

package_name = 'obj_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),        
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "obj_serial = obj_detect.obj_serial:main",
            "obj_camd = obj_detect.obj_camd:main",
            "obj_video_dumper = obj_detect.obj_vid_dumper:main",
        ],
    },
)
