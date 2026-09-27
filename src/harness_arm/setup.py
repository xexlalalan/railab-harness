from setuptools import setup
import os
from glob import glob

package_name = "harness_arm"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bunny-harness",
    maintainer_email="xexlalalan@gmail.com",
    description="PIPER driver node",
    license="Proprietary",
    entry_points={"console_scripts": ["piper_node = harness_arm.piper_node:main"]},
)
