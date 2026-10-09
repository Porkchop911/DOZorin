from setuptools import setup

name = "hwr_arm"
setup(name=name, version="0.0.1", packages=[name],
      data_files=[("share/ament_index/resource_index/packages", [f"resource/{name}"]),
                  (f"share/{name}", ["package.xml"]),
                  (f"share/{name}/config", ["config/poses.yaml"]),
                  (f"share/{name}/launch", ["launch/arm.launch.py"])],
      install_requires=["setuptools", "PyYAML"], zip_safe=True,
      description="HWR arm behavior services.", license="MIT",
      entry_points={"console_scripts": ["arm_node = hwr_arm.arm_node:main"]})
