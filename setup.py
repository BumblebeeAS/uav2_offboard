import os
from glob import glob

from setuptools import find_packages, setup

package_name = "uav2_offboard"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="monkescripts",
    maintainer_email="shaolianghe0.0@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "offboard_node = uav2_offboard.offboard_node:main",
            "go_to_position_action_server = uav2_offboard.go_to_position_action_server:main",
        ],
    },
)
