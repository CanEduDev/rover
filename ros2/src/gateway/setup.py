import os
from glob import glob

from setuptools import find_packages, setup

package_name = "gateway"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),  # noqa: PTH207, PTH118
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "battery_node = gateway.battery_node:main",
            "controller_node = gateway.controller_node:main",
            "mayor_node = gateway.mayor_node:main",
            "obstacle_detector_node = gateway.obstacle_detector_node:main",
            "wheel_node = gateway.wheel_node:main",
        ],
    },
)
