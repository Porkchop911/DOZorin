from setuptools import setup

package_name = "hwr_base"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/bringup.launch.py"]),
        (f"share/{package_name}/config", ["config/calibration.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="cmd_vel pipeline and model odometry.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "base_node = hwr_base.base_node:main",
        ],
    },
)
