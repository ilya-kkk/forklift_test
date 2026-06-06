from glob import glob
from setuptools import setup

package_name = "paette_docking_no_camera"

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
    description="Nav2-driven pallet docking from AprilTag TF.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "palette_docking_no_camera = paette_docking_no_camera.palette_docking_no_camera:main",
        ]
    },
)
