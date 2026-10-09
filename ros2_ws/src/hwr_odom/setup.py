from setuptools import setup

package_name = "hwr_odom"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/odometry.launch.py"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    description="Scan-matching and fused odometry for HWR.",
    license="MIT",
    entry_points={"console_scripts": ["odom_node = hwr_odom.odom_node:main"]},
)
