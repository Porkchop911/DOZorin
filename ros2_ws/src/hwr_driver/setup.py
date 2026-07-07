from setuptools import setup

package_name = "hwr_driver"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/driver.launch.py"]),
        (f"share/{package_name}/config", ["config/driver.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Hardware driver node over hwr_core.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "driver_node = hwr_driver.driver_node:main",
        ],
    },
)
