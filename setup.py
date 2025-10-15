from setuptools import setup

package_name = 'rl_drone'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', [
            'launch/offboard.launch.py',
            'launch/sensor_combined_listener.launch.py',
            'launch/offboard_control_launch.yaml'
        ]),
        ('share/' + package_name + '/launch', [
            'launch/test.launch.py',
            'launch/sensor_combined_listener.launch.py',
            'launch/offboard_control_launch.yaml'
        ]),
        ('share/' + package_name + '/launch', [
            'launch/train.launch.py',
            'launch/sensor_combined_listener.launch.py',
            'launch/offboard_control_launch.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='your@email.com',
    description='RL offboard control for PX4 using ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_control = rl_drone.offboard_control:main',
            'ddpg_train = rl_drone.main_ddpg_train:main',
            'test = rl_drone.test:main',
            'train = rl_drone.main_ddpg_train_t:main'
        ],
    },
)

