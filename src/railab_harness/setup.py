from setuptools import setup

package_name = "railab_harness"
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
    description="The generic middle layer: Module base, Gateway, TCP front (module interface)",
    license="Proprietary",
    # no console scripts: the process is tools/module_gateway.py (as tools/gateway_server.py is today's);
    # the adapter nodes' entry points live in harness_modules
)
