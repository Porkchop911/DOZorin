from setuptools import setup

name = "hwr_gateway"
setup(name=name, version="0.0.1", packages=[name],
      data_files=[("share/ament_index/resource_index/packages", [f"resource/{name}"]),
                  (f"share/{name}", ["package.xml"]),
                  (f"share/{name}/launch", ["launch/gateway.launch.py",
                                             "launch/autonomy.launch.py"])],
      install_requires=["setuptools", "fastapi", "uvicorn", "PyYAML"],
      zip_safe=True, description="HWR authenticated HTTP gateway.", license="MIT",
      entry_points={"console_scripts": ["gateway_node = hwr_gateway.gateway_node:main"]})
