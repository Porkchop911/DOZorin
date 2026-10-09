from setuptools import setup

package_name = "hwr_sensors"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/lidar.launch.py",
                                           "launch/camera.launch.py",
                                           "launch/sensors.launch.py"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    description="Lidar bring-up + our scan band-pass filter.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "scan_filter_node = hwr_sensors.scan_filter_node:main",
            "depth_to_ranges_node = hwr_sensors.depth_to_ranges_node:main",
            "imu_filter_node = hwr_sensors.imu_filter_node:main",
            "sensor_supervisor_node = hwr_sensors.sensor_supervisor_node:main",
        ],
    },
)
