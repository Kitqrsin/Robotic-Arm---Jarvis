from setuptools import setup
import os
from glob import glob

package_name = "arm_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="antonia",
    maintainer_email="antonia@todo.todo",
    description="ROS2 controller for 6-DOF robotic arm",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "servo_node = arm_controller.servo_node:main",
            "joint_state_publisher_node = arm_controller.joint_state_publisher_node:main",
            "cartesian_pose_service = arm_controller.cartesian_pose_service:main",
            "motor_calibration = arm_controller.motor_calibration:main",
            "stop_servos = arm_controller.stop_servos:main",
            "arm_gui = arm_controller.arm_gui:main",
            "trajectory_executor = arm_controller.trajectory_executor:main",
            "simple_controller = arm_controller.simple_controller:main",
            "camera_node = arm_controller.camera_node:main",
            "object_detector_node = arm_controller.object_detector_node:main",
            "pick_place_node = arm_controller.pick_place_node:main",
            "camera_calibration = arm_controller.camera_calibration:main",
        ],
    },
)
