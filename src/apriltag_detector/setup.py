from glob import glob

from setuptools import setup

package_name = "apriltag_detector"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/scripts", glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="codex@example.com",
    description="AprilTag detector integration for the forklift stack.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "base_link_tag_tf = apriltag_detector.base_link_tag_tf:main",
            "detection_monitor = apriltag_detector.detection_monitor:main",
            "direct_detector = apriltag_detector.direct_detector:main",
        ]
    },
)
