from setuptools import find_packages, setup

package_name = "kiwi_serial_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=[
        "setuptools",
        "pyserial",
    ],
    zip_safe=True,
    maintainer="Javier Briceno",
    maintainer_email="jbriceno225@gmail.com",
    description="ROS 2 stamped velocity and odometry bridge for KiwiDrive",
    license="MIT",
    entry_points={
        "console_scripts": [
            "cmd_vel_bridge = kiwi_serial_bridge.cmd_vel_bridge:main",
        ],
    },
)
