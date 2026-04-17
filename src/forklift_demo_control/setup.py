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
    description="Control nodes for the forklift route-server navigation stack.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cmd_vel_to_motors = forklift_demo_control.cmd_vel_to_motors:main",
            "fork_position_controller = forklift_demo_control.fork_position_controller:main",
            "map_service = forklift_demo_control.map_service:main",
            "route_graph_builder = forklift_demo_control.route_graph_builder:main",
            "route_service = forklift_demo_control.route_service:main",
            "scan_sector_filter = forklift_demo_control.scan_sector_filter:main",
        ]
    },
)
