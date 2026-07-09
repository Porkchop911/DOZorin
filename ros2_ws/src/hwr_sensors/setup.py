from setuptools import setup

package_name = "hwr_sensors"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/lidar.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Lidar bring-up + our scan band-pass filter.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "scan_filter_node = hwr_sensors.scan_filter_node:main",
        ],
    },
)
