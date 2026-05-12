from glob import glob
from setuptools import setup

package_name = "rviz"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="codex@example.com",
    description="RViz configuration and helper visualization nodes for the forklift stack.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "json_map_visualizer = rviz.json_map_visualizer:main",
            "up_lidar_marker_service = rviz.up_lidar_marker_service:main",
            "cmd_vel_activity_service = rviz.cmd_vel_activity_service:main",
            "cmd_vel_twist_stamper = rviz.cmd_vel_twist_stamper:main",
        ]
    },
)
