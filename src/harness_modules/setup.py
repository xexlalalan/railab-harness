from setuptools import setup

package_name = "harness_modules"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bunny-harness",
    maintainer_email="xexlalalan@gmail.com",
    description="the module interface's adapter nodes: arm, Bunny, camera",
    license="Proprietary",
    entry_points={"console_scripts": [
        "arm_module = harness_modules.arm_module:main",
        "bunny_module = harness_modules.bunny_module:main",
        "camera_module = harness_modules.camera_module:main",
    ]},
)
