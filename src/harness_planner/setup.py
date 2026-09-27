from setuptools import setup
p = "harness_planner"
setup(name=p, version="0.1.0", packages=[p],
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + p]),
                  ("share/" + p, ["package.xml"])],
      install_requires=["setuptools"], zip_safe=True, maintainer="bunny-harness",
      maintainer_email="xexlalalan@gmail.com", description="compound operations", license="Proprietary",
      entry_points={"console_scripts": []})
