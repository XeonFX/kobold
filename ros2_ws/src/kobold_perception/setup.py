from glob import glob

from setuptools import find_packages, setup

package_name = "kobold_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kristijan Pawlow",
    maintainer_email="kristpav@gmail.com",
    description="Camera capture and NPU object detection for Kobold.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_node = kobold_perception.camera_node:main",
            "detector_node = kobold_perception.detector_node:main",
        ],
    },
)
