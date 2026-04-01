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
            "cmd_vel_to_tricycle = forklift_demo_control.cmd_vel_to_tricycle:main",
            "hardcoded_route_sender = forklift_demo_control.hardcoded_route_sender:main",
        ]
    },
)
