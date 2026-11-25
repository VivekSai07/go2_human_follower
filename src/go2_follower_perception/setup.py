from setuptools import setup

package_name = "go2_follower_perception"
setup(name=package_name, version="0.1.0", packages=[package_name],
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                  ("share/" + package_name, ["package.xml"])],
      install_requires=["setuptools"], zip_safe=True,
      maintainer="Project Maintainer", maintainer_email="maintainer@example.com",
      description="Go2 follower perception", license="MIT",
      entry_points={"console_scripts": [
          "camera_adapter = go2_follower_perception.camera_adapter:main",
          "lidar_adapter = go2_follower_perception.lidar_adapter:main",
          "perception_node = go2_follower_perception.perception_node:main"]})
