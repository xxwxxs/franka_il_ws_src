from setuptools import find_packages, setup
import glob

package_name = "gripper_manager"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch.py")),
    ],
    include_package_data=True,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Franka Robotics GmbH",
    maintainer_email="support@franka.de",
    description="Manages the grippers of the robot.",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "franka_gripper_client = gripper_manager.franka_gripper_client:main",
        ],
    },
)
