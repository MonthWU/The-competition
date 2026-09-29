from setuptools import find_packages, setup
import os
from glob import glob
import warnings
warnings.filterwarnings("ignore")

package_name = "qrc_skandier"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test", "launch"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="neolux",
    maintainer_email="neolux@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"qrc_cam = {package_name}.qrc_cam:main",
            f"qrc_scanner = {package_name}.qrc_scanner:main",
            f"qrc_cam_killer = {package_name}.qrc_cam_killer:main",
            f"flaskr = flaskr.app:main",
        ],
    },
)
