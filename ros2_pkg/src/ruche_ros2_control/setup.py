from glob import glob
from setuptools import find_packages, setup

package_name = "ruche_ros2_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
         glob("bringup/launch/*.launch.py")),
        ("share/" + package_name + "/config",
         glob("bringup/config/*.yaml")),
        ("share/" + package_name + "/description",
         glob("description/urdf/*.xacro")),
        ("share/" + package_name + "/description",
         glob("description/ros2_control/*.xacro")),
        ("share/" + package_name + "/description",
         glob("description/utils/*.xacro")),
        ("share/" + package_name + "/rviz",
         glob("description/rviz/*.rviz")),
        ("share/" + package_name + "/worlds",
         glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ggn0",
    maintainer_email="luigi.s1994@gmail.com",
    description="Add a robot to a Gazebo world and control its velocity",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
        ],
    },
)
