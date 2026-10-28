from setuptools import setup

package_name = "hexa_ftc_ros"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/hexa_ftc.launch.py"]),
        ("share/" + package_name + "/rviz", ["rviz/hexa.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@todo.todo",
    description="Hexacopter fault tolerant control (control re-allocation)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "plant_node = hexa_ftc_ros.plant_node:main",
            "ftc_node = hexa_ftc_ros.ftc_node:main",
        ],
    },
)
