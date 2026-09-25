"""ROS2 ament_python packaging for drone_harness."""

from glob import glob
import os

from setuptools import find_packages, setup


package_name = "drone_harness"
launch_files = [path for path in glob("launch/*") if os.path.isfile(path)]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests", "tests.*"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), launch_files),
        (
            "bin",
            [
                "scripts/drone_harness_sim",
                "scripts/drone_harness_real",
            ],
        ),
    ],
    package_data={
        "drone_harness.config": ["profiles/*.yaml"],
    },
    include_package_data=True,
    install_requires=[
        "setuptools",
        "numpy>=1.24.4,<2",
        "openai>=1.0",
        "PyYAML>=6.0",
        "socksio==1.*",
    ],
    zip_safe=True,
    maintainer="hw",
    maintainer_email="toplaya@126.com",
    description="Single-target RGB-D UAV harness based on ROS2, MAVROS, and PX4.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "drone_harness_sim = drone_harness.cli:main_sim",
            "drone_harness_real = drone_harness.cli:main_real",
        ],
    },
)
