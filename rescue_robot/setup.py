from setuptools import find_packages, setup

package_name = 'rescue_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/camera_params.yaml',
            'config/robot_params.yaml',
            'config/yolov8n_bpu.yaml',
            'config/depth_anything_v2_bpu.yaml',
            'config/depth_vits_392x518_v124.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/rescue_robot.launch.py',
            'launch/rescue_robot_monocular.launch.py',
            'launch/mipi_cam_gs130w.launch.py',
        ]),
        ('share/' + package_name + '/urdf', [
            'urdf/rescue_robot.urdf',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rescue_robot',
    maintainer_email='developer@example.com',
    description='基于RDK单目视觉的塌方废墟救援寻路机器人 - 纯视觉感知方案',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = rescue_robot.camera_node:main',
            'body_detector_node = rescue_robot.body_detector_node:main',
            'depth_estimator_node = rescue_robot.depth_estimator_node:main',
            'visualization_node = rescue_robot.visualization_node:main',
            'camera_servo_node = rescue_robot.camera_servo_node:main',
        ],
    },
)
