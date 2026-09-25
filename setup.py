"""ROS2 ament_python packaging for drone_agent."""

from glob import glob
import os

from setuptools import find_packages, setup


package_name = "drone_agent"
launch_files = [path for path in glob("launch/*") if os.path.isfile(path)]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests", "tests.*"]),
    scripts=["scripts/camera_view_sim"],
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
                "scripts/drone_agent_sim",
                "scripts/drone_agent_real",
            ],
        ),
    ],
    package_data={
        "drone_agent.config": ["profiles/*.yaml"],
        "drone_agent.skills": ["*/SKILL.md"],
    },
    include_package_data=True,
    install_requires=[
        "setuptools",
        "dds-cloudapi-sdk==0.5.3",
        "Flask==2.3.3",
        "numpy>=1.24.4,<2",
        "openai>=1.0",
        "PyYAML>=6.0",
        "socksio==1.*",
    ],
    zip_safe=True,
    maintainer="hw",
    maintainer_email="toplaya@126.com",
    description="Natural-language UAV control agent based on ROS2, MAVROS, and PX4.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "drone_agent_sim = drone_agent.cli:main_sim",
            "drone_agent_real = drone_agent.cli:main_real",
        ],
    },
)
