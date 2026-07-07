from setuptools import setup

package_name = "hwr_teleop"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Keyboard teleop.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "teleop_node = hwr_teleop.teleop_node:main",
        ],
    },
)
