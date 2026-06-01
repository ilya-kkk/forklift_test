from glob import glob
from setuptools import setup

package_name = "robot_control_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="codex@example.com",
    description="VDA-independent robot mission control core for the forklift stack.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "robot_control_core = robot_control_core.robot_control_core:main",
        ]
    },
)
