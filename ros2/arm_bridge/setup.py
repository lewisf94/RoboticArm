from setuptools import find_packages, setup

package_name = "arm_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lewis Fowler",
    maintainer_email="lewisfowler94@gmail.com",
    description="ROS 2 <-> RoboticArm serial bridge (translator only).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge = arm_bridge.bridge:main",
        ],
    },
)
