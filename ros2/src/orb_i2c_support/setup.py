from setuptools import setup

package_name = "orb_i2c_support"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/i2c_manager.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TomorrowTech",
    maintainer_email="sviluppo@tomorrowtech.it",
    description="ROS 2 utilities for interacting with the ORB I2C controller.",
    license="CC BY-NC 4.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "i2c_manager = orb_i2c_support.i2c_node:main",
        ],
    },
)
