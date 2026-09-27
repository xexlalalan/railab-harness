from setuptools import setup
p = "harness_calibration"
setup(name=p, version="0.1.0", packages=[p],
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + p]),
                  ("share/" + p, ["package.xml"])],
      install_requires=["setuptools"], zip_safe=True, maintainer="bunny-harness",
      maintainer_email="xexlalalan@gmail.com", description="calibration + teach tools", license="Proprietary",
      entry_points={"console_scripts": ["handeye = harness_calibration.handeye:main",
                                        "teach = harness_calibration.teach_cli:main",
                                        "jog = harness_calibration.jog:main",
                                        "center = harness_calibration.center:main",
                                        "floor = harness_calibration.floor:main",
                                        "locate = harness_calibration.locate:main"]})
