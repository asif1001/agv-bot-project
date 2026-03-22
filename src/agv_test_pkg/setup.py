from setuptools import find_packages, setup

package_name = 'agv_test_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asif',
    maintainer_email='asif@todo.todo',
    description='Beginner-friendly ROS 2 Python package that publishes AGV status messages.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_agv_node = agv_test_pkg.hello_agv_node:main',
            'agv_listener_node = agv_test_pkg.agv_listener_node:main',
            'agv_keyboard_teleop = agv_test_pkg.agv_keyboard_teleop:main',
            'agv_goal_nav = agv_test_pkg.agv_goal_nav:main',
            'agv_nav_goal_bridge = agv_test_pkg.agv_nav_goal_bridge:main',
            'web_sensor_dashboard = agv_test_pkg.web_sensor_dashboard:main',
        ],
    },
)
