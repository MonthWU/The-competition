from setuptools import setup

package_name = "web_tuner_node"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "Flask"],
    zip_safe=True,
    maintainer="MonthWU",
    maintainer_email="monthwu@example.com",
    description="ROS2 Flask node for camera preview, status, and tuning operations.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "web_tuner_node = web_tuner_node.web_tuner_node:main",
        ],
    },
)
