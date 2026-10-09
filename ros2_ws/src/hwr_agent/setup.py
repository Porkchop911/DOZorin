from setuptools import setup

name = "hwr_agent"
setup(name=name, version="0.0.1", packages=[name],
      data_files=[("share/ament_index/resource_index/packages", [f"resource/{name}"]),
                  (f"share/{name}", ["package.xml"]),
                  (f"share/{name}/config", ["config/tools.json"])],
      install_requires=["setuptools", "requests"], zip_safe=True,
      description="Guarded HWR agent skills.", license="MIT",
      entry_points={"console_scripts": [
          "hwr_mission = hwr_agent.mission:main",
          "hwr_mcp = hwr_agent.mcp_server:main",
      ]})
