from setuptools import setup
import os
from glob import glob
p = "harness_vision"
setup(name=p, version="0.1.0", packages=[p],
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + p]),
                  ("share/" + p, ["package.xml"]),
                  (os.path.join("share", p, "config"), glob("config/*.yaml")),
                  (os.path.join("share", p, "launch"), glob("launch/*.py"))],
      install_requires=["setuptools"], zip_safe=True, maintainer="bunny-harness",
      maintainer_email="xexlalalan@gmail.com", description="camera + AprilTag node", license="Proprietary",
      entry_points={"console_scripts": ["tag_node = harness_vision.tag_node:main",
                                        "mjpeg_server = harness_vision.mjpeg_server:main"]})
