from setuptools import find_packages, setup

setup(
    name="car26_system", version="0.1.0", packages=find_packages(),
    package_data={"car26.hmi": ["index.html"]},
    data_files=[("share/ament_index/resource_index/packages", ["resource/car26_system"]),
                ("share/car26_system", ["package.xml"]),
                ("share/car26_system/launch", ["launch/car26.launch.py"])],
    install_requires=["setuptools"], zip_safe=False,
    maintainer="Monthwu", maintainer_email="monthwu@example.com",
    description="CAR26 four-module application", license="Proprietary",
    entry_points={"console_scripts": ["car26 = car26.cli:main"]},
)
