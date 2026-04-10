from setuptools import setup


package_name = "forklift_demo_control"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="codex@example.com",
    description="Control nodes for the forklift FollowPath demo.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "keyboard_teleop = forklift_demo_control.keyboard_teleop:main",
            "cmd_vel_to_motors = forklift_demo_control.cmd_vel_to_motors:main",
            "hardcoded_route_sender = forklift_demo_control.hardcoded_route_sender:main",
            "map_service = forklift_demo_control.map_service:main",
            "route_service = forklift_demo_control.route_service:main",
            "scan_sector_filter = forklift_demo_control.scan_sector_filter:main",
            "demo_route_loop = forklift_demo_control.demo_route_loop:main",
            "rviz_teleop_marker = forklift_demo_control.rviz_teleop_marker:main",
        ]
    },
)
