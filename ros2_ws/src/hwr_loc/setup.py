from setuptools import setup

name = "hwr_loc"
setup(name=name, version="0.0.1", packages=[name],
      data_files=[("share/ament_index/resource_index/packages", [f"resource/{name}"]),
                  (f"share/{name}", ["package.xml"]),
                  (f"share/{name}/launch", ["launch/localization.launch.py"])],
      install_requires=["setuptools", "numpy", "PyYAML"], zip_safe=True,
      description="HWR Monte-Carlo localization.", license="MIT",
      entry_points={"console_scripts": ["localizer_node = hwr_loc.localizer_node:main"]})
