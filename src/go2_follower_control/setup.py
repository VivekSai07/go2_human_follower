from setuptools import setup

package_name = "go2_follower_control"
setup(name=package_name, version="0.1.0", packages=[package_name],
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                  ("share/" + package_name, ["package.xml"])],
      install_requires=["setuptools"], zip_safe=True,
      maintainer="Project Maintainer", maintainer_email="maintainer@example.com",
      description="Go2 follower control", license="MIT",
      entry_points={"console_scripts": ["control_node = go2_follower_control.control_node:main"]})
