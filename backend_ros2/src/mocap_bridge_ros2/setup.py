from setuptools import find_packages, setup

package_name = "mocap_bridge_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/mocap_bridge.launch.py"]),
        (f"share/{package_name}/config", ["config/mocap.yaml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@todo",
    description="Minimal OptiTrack MoCap bridge (VRPN) publishing ROS2 PoseStamped topics.",
    license="MIT",
    entry_points={"console_scripts": ["mocap_bridge_node = mocap_bridge_ros2.node:main"]},
)