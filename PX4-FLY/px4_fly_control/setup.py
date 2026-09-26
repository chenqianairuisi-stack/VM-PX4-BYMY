from setuptools import setup

package_name = 'px4_fly_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PX4-FLY',
    maintainer_email='local@example.com',
    description='Keyboard Offboard control for the PX4-FLY Gazebo Classic quadrotor.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'keyboard_control = px4_fly_control.keyboard_control:main',
            'safe_visual_follow = px4_fly_control.safe_visual_follow:main',
            'yolo_detector = px4_fly_control.yolo_detector:main',
            'red_color_detector = px4_fly_control.red_color_detector:main',
        ],
    },
)
