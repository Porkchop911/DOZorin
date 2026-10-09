from setuptools import setup

name = "hwr_nav"
setup(name=name, version="0.0.1", packages=[name],
      data_files=[("share/ament_index/resource_index/packages", [f"resource/{name}"]),
                  (f"share/{name}", ["package.xml"]),
                  (f"share/{name}/launch", ["launch/navigation.launch.py"]),
                  (f"share/{name}/config", ["config/places.yaml"])],
      install_requires=["setuptools", "numpy", "PyYAML"], zip_safe=True,
      description="HWR A-star navigation and holonomic controller.", license="MIT",
      entry_points={"console_scripts": ["nav_node = hwr_nav.nav_node:main"]})
