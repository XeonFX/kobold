from setuptools import find_packages, setup

package_name = "kobold_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Kobold maintainers",
    maintainer_email="kristpav@gmail.com",
    description="Serial bridge between the kobold ESP32 boards and ROS 2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge = kobold_bridge.bridge_node:main",
            # Fake drive/sense boards on PTY devices — lets the whole stack run
            # with no hardware attached.
            "sim = kobold_bridge.sim:main",
        ],
    },
)
