from setuptools import setup

name = "hwr_explore"
setup(name=name, version="0.0.1", packages=[name],
      data_files=[("share/ament_index/resource_index/packages", [f"resource/{name}"]),
                  (f"share/{name}", ["package.xml"]),
                  (f"share/{name}/launch", ["launch/exploration.launch.py"])],
      install_requires=["setuptools", "numpy"], zip_safe=True,
      description="HWR frontier exploration orchestrator.", license="MIT",
      entry_points={"console_scripts": ["explore_node = hwr_explore.explore_node:main"]})
