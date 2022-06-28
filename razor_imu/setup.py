import os
from glob import glob
from setuptools import setup

package_name = "razor_imu"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (os.path.join("share", package_name), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tannerkoza",
    maintainer_email="jtk0018@auburn.edu",
    description="This package parses the serial output of a Sparkfun Open Log Artemis IMU and publishes a corresponding IMU topic.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["razor_imu_node = razor_imu.razor_imu_node:main"],
    },
)
