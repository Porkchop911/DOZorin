from setuptools import setup

package_name = "hwr_description"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/urdf", ["urdf/hwr.urdf"]),
        (f"share/{package_name}/launch", ["launch/display.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="URDF + state publishers.",
    license="MIT",
)
