from glob import glob
from setuptools import setup

package_name = "go2_follower_bringup"
setup(name=package_name, version="0.1.0", packages=[package_name],
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                  ("share/" + package_name, ["package.xml"]),
                  ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
                  ("share/" + package_name + "/config", glob("config/*.yaml"))],
      install_requires=["setuptools"], zip_safe=True,
      maintainer="Project Maintainer", maintainer_email="maintainer@example.com",
      description="Go2 follower bringup", license="MIT")
