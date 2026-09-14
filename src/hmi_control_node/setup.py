from setuptools import setup

package_name = "hmi_control_node"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MonthWU",
    maintainer_email="monthwu@example.com",
    description="HMI command parser and response node for the Jetson vision tuning screen.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hmi_control_node = hmi_control_node.hmi_control_node:main",
        ],
    },
)
