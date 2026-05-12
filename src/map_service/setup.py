from glob import glob
from setuptools import setup

package_name = "map_service"

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
    description="Forklift module package: map_service.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "map_service = map_service.map_service:main",
        ]
    },
)
